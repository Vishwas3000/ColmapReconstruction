param(
    [Parameter(Mandatory=$true)]
    [string]$workspace,
    [ValidateSet("dense", "sparse", "dense_aligned", "sparse_aligned")]
    [string]$type = "dense"
)

$map = @{
    "dense"          = "dense\fused.ply"
    "sparse"         = "sparse\sparse.ply"
    "dense_aligned"  = "dense\fused_aligned.ply"
    "sparse_aligned" = "sparse\sparse_aligned.ply"
}

$ply = "$workspace\$($map[$type])"

if (-not (Test-Path $ply)) {
    Write-Error "PLY not found at: $ply"
    exit 1
}

& "C:\Users\timeu\OneDrive\Documents\GitHub\meshlab\meshlab.exe" $ply
