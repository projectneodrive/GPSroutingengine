# Download an OSM extract of all highway ways in a bounding box
# via the Overpass API, saved as data/map.osm.
#
# Usage:  .\fetch_osm.ps1 [-South 46.508] [-West 6.58] [-North 46.545] [-East 6.65]

param(
    [double]$South = 46.508,
    [double]$West  = 6.58,
    [double]$North = 46.545,
    [double]$East  = 6.65,
    [string]$Out   = "$PSScriptRoot\map.osm"
)

$bbox = "$South,$West,$North,$East"
$query = @"
[out:xml][timeout:120];
(way["highway"]($bbox););
(._;>;);
out body;
"@

Write-Host "Querying Overpass for bbox $bbox ..."
$body = "data=" + [uri]::EscapeDataString($query)
curl.exe -s -X POST -d $body "https://overpass-api.de/api/interpreter" -o $Out
if ($LASTEXITCODE -ne 0) { Write-Error "Overpass request failed"; exit 1 }
$size = (Get-Item $Out).Length / 1MB
Write-Host ("Saved {0} ({1:N1} MB)" -f $Out, $size)
