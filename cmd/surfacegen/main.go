// cmd/surfacegen/main.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

// surfacegen generates the airport surface data in resources/surface that
// tower positions and the ASDE-X display use. The taxi network, runways
// and gates come from an X-Plane apt.dat file, either a local one or the
// airport's recommended scenery from the X-Plane Scenery Gateway (whose
// airports are licensed under the GPL, version 2 or later). Surface
// polygons come from a CRC installation's ASDE-X video map if one is
// given; CRC files are only read, never modified. Otherwise the runway
// outlines are synthesized.
//
// Example:
//
//	go run ./cmd/surfacegen -icao KCOS -gateway -crc %LOCALAPPDATA%\CRC -artcc ZDV -facility COS
package main

import (
	"archive/zip"
	"bytes"
	"encoding/base64"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"slices"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/aviation/db"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/surface"
)

const gatewayAPI = "https://gateway.x-plane.com/apiv1"

func main() {
	var icao, aptPath, crcDir, artcc, facility, outPath string
	var gateway bool
	flag.StringVar(&icao, "icao", "", "ICAO code of the airport (e.g. KCOS)")
	flag.StringVar(&aptPath, "apt", "", "apt.dat file to read the airport from")
	flag.BoolVar(&gateway, "gateway", false, "Download the airport's recommended scenery from the X-Plane Scenery Gateway")
	flag.StringVar(&crcDir, "crc", "", "CRC installation directory to read the ASDE-X map from (read only)")
	flag.StringVar(&artcc, "artcc", "", "ARTCC whose CRC data has the ASDE-X configuration (with -crc)")
	flag.StringVar(&facility, "facility", "", "CRC facility with the ASDE-X configuration (with -crc)")
	flag.StringVar(&outPath, "out", "", "Output file (default resources/surface/<ICAO>.json.zst)")
	flag.Parse()

	log.SetFlags(0)
	if icao == "" {
		log.Fatal("-icao is required")
	}
	if (aptPath == "") == !gateway {
		log.Fatal("exactly one of -apt and -gateway must be given")
	}
	if crcDir != "" && (artcc == "" || facility == "") {
		log.Fatal("-artcc and -facility are required with -crc")
	}
	if outPath == "" {
		outPath = filepath.Join("resources", filepath.FromSlash(surface.ResourcePath(icao)))
	}

	var aptDat []byte
	var source string
	if gateway {
		var err error
		if aptDat, source, err = fetchGatewayAptDat(icao); err != nil {
			log.Fatalf("%s: %v", icao, err)
		}
	} else {
		var err error
		if aptDat, err = os.ReadFile(aptPath); err != nil {
			log.Fatal(err)
		}
		source = "apt.dat: " + filepath.Base(aptPath)
	}

	ap, err := surface.ParseAptDat(bytes.NewReader(aptDat), icao)
	if err != nil {
		log.Fatal(err)
	}
	ap.Sources = append(ap.Sources, source)

	matchCIFPRunways(ap)

	if crcDir != "" {
		asdex, err := surface.ReadCRCASDEX(crcDir, artcc, facility)
		if err != nil {
			log.Fatalf("CRC: %v", err)
		}
		ap.Polygons = asdex.Polygons
		ap.TowerLocation = asdex.TowerLocation
		ap.Sources = append(ap.Sources, fmt.Sprintf("CRC %s %s ASDE-X video map %q", artcc, facility, asdex.MapName))
		log.Printf("Read %d ASDE-X polygons from CRC map %q", len(asdex.Polygons), asdex.MapName)
	} else {
		for _, rwy := range ap.Runways {
			outline := ap.RunwayOutline(rwy)
			ap.Polygons = append(ap.Polygons, surface.Polygon{
				Kind:  surface.PolygonRunway,
				Rings: [][]math.Point2LL{outline[:]},
			})
		}
		log.Printf("No CRC ASDE-X map given; synthesized %d runway outlines", len(ap.Runways))
	}
	slices.SortStableFunc(ap.Polygons, func(a, b surface.Polygon) int {
		return a.Kind.DrawOrder() - b.Kind.DrawOrder()
	})

	if err := ap.Finalize(); err != nil {
		log.Fatal(err)
	}
	checkConnectivity(ap)

	if err := os.MkdirAll(filepath.Dir(outPath), 0o755); err != nil {
		log.Fatal(err)
	}
	f, err := os.Create(outPath)
	if err != nil {
		log.Fatal(err)
	}
	if err := ap.Encode(f); err != nil {
		log.Fatal(err)
	}
	if err := f.Close(); err != nil {
		log.Fatal(err)
	}
	log.Printf("Wrote %s: %d runways, %d taxi nodes, %d taxi edges, %d gates, %d polygons",
		outPath, len(ap.Runways), len(ap.Nodes), len(ap.Edges), len(ap.Gates), len(ap.Polygons))
}

// fetchGatewayAptDat downloads the airport's recommended scenery pack
// from the X-Plane Scenery Gateway and returns its apt.dat and a
// description of its source for attribution.
func fetchGatewayAptDat(icao string) ([]byte, string, error) {
	var airport struct {
		Airport struct {
			RecommendedSceneryId int `json:"recommendedSceneryId"`
		} `json:"airport"`
	}
	if err := getJSON(gatewayAPI+"/airport/"+icao, &airport); err != nil {
		return nil, "", err
	}
	id := airport.Airport.RecommendedSceneryId
	if id == 0 {
		return nil, "", fmt.Errorf("no recommended scenery on the Gateway")
	}

	var scenery struct {
		Scenery struct {
			SceneryId     int    `json:"sceneryId"`
			UserName      string `json:"userName"`
			DateApproved  string `json:"dateApproved"`
			MasterZipBlob string `json:"masterZipBlob"`
		} `json:"scenery"`
	}
	if err := getJSON(fmt.Sprintf("%s/scenery/%d", gatewayAPI, id), &scenery); err != nil {
		return nil, "", err
	}
	zipBytes, err := base64.StdEncoding.DecodeString(scenery.Scenery.MasterZipBlob)
	if err != nil {
		return nil, "", fmt.Errorf("scenery %d: %w", id, err)
	}
	zr, err := zip.NewReader(bytes.NewReader(zipBytes), int64(len(zipBytes)))
	if err != nil {
		return nil, "", fmt.Errorf("scenery %d: %w", id, err)
	}
	// The scenery zip holds the airport's apt.dat as <ICAO>.dat.
	zf, err := zr.Open(icao + ".dat")
	if err != nil {
		return nil, "", fmt.Errorf("scenery %d: %w", id, err)
	}
	defer zf.Close()
	b, err := io.ReadAll(zf)
	if err != nil {
		return nil, "", err
	}

	log.Printf("Downloaded Gateway scenery %d by %s", id, scenery.Scenery.UserName)
	return b, fmt.Sprintf("X-Plane Scenery Gateway scenery %d by %s, approved %s; GPL version 2 or later",
		id, scenery.Scenery.UserName, scenery.Scenery.DateApproved), nil
}

func getJSON(url string, v any) error {
	client := http.Client{Timeout: time.Minute}
	resp, err := client.Get(url)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("%s: %s", url, resp.Status)
	}
	return json.NewDecoder(resp.Body).Decode(v)
}

// matchCIFPRunways matches the apt.dat runway ends to the runways in the
// FAA CIFP that the rest of vice uses, renaming apt.dat runway ends whose
// identifiers differ (e.g., after a runway has been renumbered).
func matchCIFPRunways(ap *surface.Airport) {
	db.InitDB()
	dbap, ok := db.DB.Airports[av.ICAOAirportCode(ap.ICAO)]
	if !ok {
		log.Printf("%s: not in the CIFP; runway identifiers not checked", ap.ICAO)
		return
	}

	// CIFP thresholds are landing thresholds, so compare against the
	// apt.dat thresholds moved down the runway by any displacement.
	type end struct {
		id        string
		threshold math.Point2LL
	}
	var ends []end
	for _, rwy := range ap.Runways {
		for i, e := range rwy.Ends {
			p0, p1 := ap.Local(e.Threshold), ap.Local(rwy.Ends[1-i].Threshold)
			d := e.DisplacedThresholdFt * math.FeetToNauticalMiles
			p := math.Add2f(p0, math.Scale2f(math.Normalize2f(math.Sub2f(p1, p0)), d))
			ends = append(ends, end{id: e.Id, threshold: ap.FromLocal(p)})
		}
	}

	matched := make(map[string]bool)
	for _, rwy := range dbap.Runways {
		best, bestDist := -1, float32(0)
		for i, e := range ends {
			if d := ap.Distance(e.threshold, rwy.Threshold); best == -1 || d < bestDist {
				best, bestDist = i, d
			}
		}
		if best == -1 || bestDist > 0.15 {
			log.Printf("CIFP runway %s: no apt.dat runway end within 0.15nm of its threshold", rwy.Id)
			continue
		}
		e := ends[best]
		matched[e.id] = true
		if e.id != rwy.Id {
			log.Printf("Renaming apt.dat runway %s to %s, its CIFP identifier", e.id, rwy.Id)
			ap.RenameRunwayEnd(e.id, rwy.Id)
			matched[rwy.Id] = true
		}
		if bestDist > 0.02 {
			log.Printf("Runway %s: apt.dat and CIFP thresholds are %.0f feet apart", rwy.Id,
				bestDist*math.NauticalMilesToFeet)
		}
	}
	for _, e := range ends {
		if !matched[e.id] {
			log.Printf("apt.dat runway %s is not in the CIFP", e.id)
		}
	}
}

// checkConnectivity reports runways that can't be reached from the gates
// and vice versa, which indicates problems with the taxi network.
func checkConnectivity(ap *surface.Airport) {
	var gateNodes []surface.NodeID
	for _, g := range ap.Gates {
		if n, ok := ap.GateNode(g); ok {
			gateNodes = append(gateNodes, n)
		} else {
			log.Printf("Gate %s: no nearby taxi network node", g.Name)
		}
	}

	for _, rwy := range ap.Runways {
		for _, e := range rwy.Ends {
			entry, ok := ap.DepartureEntry(e.Id)
			if !ok {
				log.Printf("Runway %s: no taxiway connections", e.Id)
				continue
			}
			unreachable := 0
			for _, g := range gateNodes {
				if _, err := ap.FindRoute(surface.RouteRequest{From: g, To: []surface.NodeID{entry}}); err != nil {
					unreachable++
				}
			}
			if unreachable > 0 {
				log.Printf("Runway %s: departure entry unreachable from %d of %d gates", e.Id, unreachable, len(gateNodes))
			}
		}
	}
}
