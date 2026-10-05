// surface/crc.go
// Copyright(c) vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package surface

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"

	"github.com/mmp/vice/math"
)

// CRCASDEX holds what we use from a CRC facility's ASDE-X configuration.
type CRCASDEX struct {
	Facility      string
	MapName       string
	TowerLocation math.Point2LL
	Polygons      []Polygon
}

// Just the parts of the CRC ARTCC JSON that we need.
type crcFacility struct {
	Id                 string        `json:"id"`
	ChildFacilities    []crcFacility `json:"childFacilities"`
	ASDEXConfiguration *struct {
		VideoMapId    string `json:"videoMapId"`
		TowerLocation struct {
			Lat float32 `json:"lat"`
			Lon float32 `json:"lon"`
		} `json:"towerLocation"`
	} `json:"asdexConfiguration"`
}

type crcARTCC struct {
	Facility  crcFacility `json:"facility"`
	VideoMaps []struct {
		Id   string `json:"id"`
		Name string `json:"name"`
	} `json:"videoMaps"`
}

type geoJSON struct {
	Features []struct {
		Geometry struct {
			Type        string          `json:"type"`
			Coordinates json.RawMessage `json:"coordinates"`
		} `json:"geometry"`
		Properties struct {
			ASDEX string `json:"asdex"`
		} `json:"properties"`
	} `json:"features"`
}

// ReadCRCASDEX reads the ASDE-X configuration and surface polygons for
// the given facility (e.g. "COS") from the given ARTCC's data in a CRC
// installation directory (typically %LOCALAPPDATA%\CRC). Files are only
// opened for reading.
func ReadCRCASDEX(crcDir, artcc, facility string) (*CRCASDEX, error) {
	var a crcARTCC
	if err := readJSONFile(filepath.Join(crcDir, "ARTCCs", artcc+".json"), &a); err != nil {
		return nil, err
	}

	var find func(f *crcFacility) *crcFacility
	find = func(f *crcFacility) *crcFacility {
		if f.Id == facility {
			return f
		}
		for i := range f.ChildFacilities {
			if r := find(&f.ChildFacilities[i]); r != nil {
				return r
			}
		}
		return nil
	}
	fac := find(&a.Facility)
	if fac == nil {
		return nil, fmt.Errorf("%s: facility not found in %s", facility, artcc)
	}
	cfg := fac.ASDEXConfiguration
	if cfg == nil || cfg.VideoMapId == "" {
		return nil, fmt.Errorf("%s: no ASDE-X configuration", facility)
	}

	result := &CRCASDEX{
		Facility:      facility,
		TowerLocation: math.Point2LL{cfg.TowerLocation.Lon, cfg.TowerLocation.Lat},
	}
	for _, m := range a.VideoMaps {
		if m.Id == cfg.VideoMapId {
			result.MapName = m.Name
		}
	}

	var g geoJSON
	if err := readJSONFile(filepath.Join(crcDir, "VideoMaps", artcc, cfg.VideoMapId+".geojson"), &g); err != nil {
		return nil, err
	}
	for i, f := range g.Features {
		kind := PolygonKind(f.Properties.ASDEX)
		if kind.DrawOrder() > PolygonStructure.DrawOrder() {
			return nil, fmt.Errorf("feature %d: unknown ASDE-X class %q", i, f.Properties.ASDEX)
		}

		var polys [][][][2]float32 // polygon -> ring -> [lon, lat]
		switch f.Geometry.Type {
		case "Polygon":
			var p [][][2]float32
			if err := json.Unmarshal(f.Geometry.Coordinates, &p); err != nil {
				return nil, fmt.Errorf("feature %d: %w", i, err)
			}
			polys = append(polys, p)
		case "MultiPolygon":
			if err := json.Unmarshal(f.Geometry.Coordinates, &polys); err != nil {
				return nil, fmt.Errorf("feature %d: %w", i, err)
			}
		default:
			return nil, fmt.Errorf("feature %d: unexpected geometry type %q", i, f.Geometry.Type)
		}

		for _, p := range polys {
			poly := Polygon{Kind: kind}
			for _, ring := range p {
				// GeoJSON rings repeat the first point at the end.
				if len(ring) > 1 && ring[0] == ring[len(ring)-1] {
					ring = ring[:len(ring)-1]
				}
				if len(ring) < 3 {
					continue
				}
				r := make([]math.Point2LL, len(ring))
				for j, pt := range ring {
					r[j] = math.Point2LL(pt)
				}
				poly.Rings = append(poly.Rings, r)
			}
			if len(poly.Rings) > 0 {
				result.Polygons = append(result.Polygons, poly)
			}
		}
	}
	return result, nil
}

func readJSONFile(path string, v any) error {
	f, err := os.Open(path) // read-only
	if err != nil {
		return err
	}
	defer f.Close()
	if err := json.NewDecoder(f).Decode(v); err != nil {
		return fmt.Errorf("%s: %w", path, err)
	}
	return nil
}
