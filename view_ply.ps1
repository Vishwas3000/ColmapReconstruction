param(
    [Parameter(Mandatory=$true)]
    [string]$workspace,
    [ValidateSet("dense", "sparse")]
    [string]$type = "dense"
)

if ($type -eq "dense") {
    $ply = "$workspace\dense\fused.ply"
} else {
    $ply = "$workspace\sparse\sparse.ply"
}

if (-not (Test-Path $ply)) {
    Write-Error "PLY not found at: $ply"
    exit 1
}

& "C:\Users\timeu\OneDrive\Documents\GitHub\meshlab\meshlab.exe" $ply
