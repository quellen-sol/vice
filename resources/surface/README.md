# Airport surface data

Each `<ICAO>.json.zst` file describes an airport's movement area for tower
positions and the ASDE-X display: runways, the taxi routing network with
runway hold-short zones, gates, and surface polygons. See the `surface`
package for the format.

## Sources and licenses

- **Taxi network, runways and gates**: airport sceneries from the
  [X-Plane Scenery Gateway](https://gateway.x-plane.com/). Gateway
  sceneries are free software, redistributable and modifiable under the GNU
  General Public License, version 2 or (at your option) any later version.
  The scenery ID and author of each airport are recorded in the file's
  `sources` field.
- **Surface polygons**: CRC ASDE-X video maps, when available (as with the
  CRC-derived STARS and ERAM video maps in `resources/videomaps`).
  Otherwise runway outlines are synthesized from the runway data.

| Airport | Gateway scenery | CRC ASDE-X map |
|---------|-----------------|----------------|
| KCOS    | 93255 (Coloradoaviation) | ZDV COS "COS SAID" |

## Regenerating

From the repository root:

```
go run ./cmd/surfacegen -icao KCOS -gateway -crc "%LOCALAPPDATA%\CRC" -artcc ZDV -facility COS
```

`surfacegen` only reads from the CRC directory.
