param(
	[ValidateSet('debug', 'checked', 'profile', 'release')]
	[string[]] $Configuration,
	[ValidateRange(1, [int]::MaxValue)]
	[int] $Jobs
)

$ErrorActionPreference = 'Stop'
$mount = "${PSScriptRoot}:/src"
$createArguments = @('create')
if ($PSBoundParameters.ContainsKey('Jobs')) {
	$createArguments += @('--env', "JOBS=$Jobs")
}
$createArguments += @(
	'--volume', $mount
	'physx5wasm'
	'/src/docker-build.sh'
)
$createArguments += $Configuration

$containerId = & docker @createArguments
if ($LASTEXITCODE -ne 0) {
	throw 'Unable to create the PhysX build container.'
}
$transferDirectory = Join-Path ([System.IO.Path]::GetTempPath()) ("physx5wasm-" + [guid]::NewGuid())

try {
	& docker start --attach $containerId
	if ($LASTEXITCODE -ne 0) {
		throw 'The PhysX build failed.'
	}

	$dist = Join-Path $PSScriptRoot 'dist'
	New-Item -ItemType Directory -Force $transferDirectory | Out-Null
	& docker cp "${containerId}:/tmp/physx-dist/." $transferDirectory
	if ($LASTEXITCODE -ne 0) {
		throw 'Unable to copy the PhysX libraries from the build container.'
	}

	New-Item -ItemType Directory -Force $dist | Out-Null
	Copy-Item -Path (Join-Path $transferDirectory '*') -Destination $dist -Recurse -Force
}
finally {
	& docker rm --force $containerId | Out-Null
	if (Test-Path -LiteralPath $transferDirectory) {
		Remove-Item -LiteralPath $transferDirectory -Recurse -Force
	}
}
