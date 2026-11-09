<#
.SYNOPSIS
    Create the GitHub repository and push main, in one command.

.DESCRIPTION
    Creates the repository through the GitHub REST API with the description and
    topics from this project, then pushes the current branch.

    A token with `repo` scope is required. Create one at:
      https://github.com/settings/tokens/new?scopes=repo&description=uav-yolo-offboard-tracking

.PARAMETER Public
    Create the repository as public. Default is private: publish only after
    results/landing_error.csv contains real rows (docs/pre-release-checklist.md).

.PARAMETER DryRun
    Validate the local repository and the network, print what would happen,
    change nothing. Works without a token.

.EXAMPLE
    $env:GITHUB_TOKEN = "ghp_..."
    powershell -File scripts/publish.ps1

.EXAMPLE
    powershell -File scripts/publish.ps1 -DryRun
#>
param(
    [switch]$Public,
    [switch]$DryRun,
    [string]$Owner = "wuuu854-boop",
    [string]$Repo = "uav-yolo-offboard-tracking"
)

$ErrorActionPreference = "Stop"
# Windows PowerShell 5.1 still defaults to older TLS on some machines.
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root

$description = "PX4/Gazebo SITL closed loop: YOLO detection to ground projection to ROS 2 Offboard approach, with a trial harness, benchmarks and an environment doctor."
$topics = @(
    "uav", "px4", "gazebo", "ros2", "yolo",
    "onnxruntime", "offboard", "uxrce-dds", "simulation", "computer-vision"
)

function Write-Step($message) { Write-Host "==> $message" -ForegroundColor Cyan }
function Write-Ok($message) { Write-Host "    $message" -ForegroundColor Green }
function Write-Warn($message) { Write-Host "    $message" -ForegroundColor Yellow }

Write-Step "checking the local repository"
$branch = git rev-parse --abbrev-ref HEAD
if ($branch -ne "main") { throw "expected branch main, found '$branch'" }
if (git status --porcelain) { throw "working tree is dirty; commit or stash first" }
$commits = git rev-list --count HEAD
Write-Ok "$commits commits on main, clean tree"

Write-Step "checking GitHub connectivity"
try {
    $null = Invoke-RestMethod -Uri "https://api.github.com/zen" -TimeoutSec 15
    Write-Ok "api.github.com reachable"
} catch {
    throw "cannot reach api.github.com: $($_.Exception.Message)"
}

$token = $env:GITHUB_TOKEN
if (-not $token) { $token = $env:GH_TOKEN }

if ($DryRun) {
    Write-Step "dry run: nothing will be created or pushed"
    Write-Ok "target: https://github.com/$Owner/$Repo ($(if ($Public) { 'public' } else { 'private' }))"
    Write-Ok "description: $description"
    Write-Ok "topics: $($topics -join ', ')"
    if ($token) { Write-Ok "token found (length $($token.Length))" } else { Write-Warn "no GITHUB_TOKEN set; the real run will stop here" }
    exit 0
}

if (-not $token) {
    throw @"
No token. Either:
  `$env:GITHUB_TOKEN = "ghp_..."   # https://github.com/settings/tokens/new?scopes=repo
  powershell -File scripts/publish.ps1
or create the repository by hand:
  https://github.com/new  ->  name: $Repo  ->  do not initialise anything
  git remote add origin https://github.com/$Owner/$Repo.git
  git push -u origin main
"@
}

$headers = @{
    Authorization          = "Bearer $token"
    Accept                 = "application/vnd.github+json"
    "X-GitHub-Api-Version" = "2022-11-28"
}

Write-Step "creating the repository"
$payload = @{
    name           = $Repo
    description    = $description
    private        = -not $Public
    has_issues     = $true
    has_projects   = $true
    has_wiki       = $false
    has_discussions = $false
    auto_init      = $false
} | ConvertTo-Json

try {
    $null = Invoke-RestMethod -Method Post -Uri "https://api.github.com/user/repos" `
        -Headers $headers -Body $payload -ContentType "application/json" -TimeoutSec 30
    Write-Ok "created"
} catch {
    $status = $_.Exception.Response.StatusCode.value__
    if ($status -eq 422) {
        Write-Warn "repository already exists, continuing"
    } else {
        throw "create failed (HTTP $status): $($_.ErrorDetails.Message)"
    }
}

Write-Step "setting description and topics"
$null = Invoke-RestMethod -Method Patch -Uri "https://api.github.com/repos/$Owner/$Repo" `
    -Headers $headers -Body (@{ description = $description; has_wiki = $false } | ConvertTo-Json) `
    -ContentType "application/json" -TimeoutSec 30
$null = Invoke-RestMethod -Method Put -Uri "https://api.github.com/repos/$Owner/$Repo/topics" `
    -Headers $headers -Body (@{ names = $topics } | ConvertTo-Json) `
    -ContentType "application/json" -TimeoutSec 30
Write-Ok "$($topics.Count) topics set"

Write-Step "pushing main"
# The user-level git config points at a local proxy that is often not running;
# bypass it for this push only.
git remote remove origin 2>$null
git remote add origin "https://github.com/$Owner/$Repo.git"
git -c http.proxy= -c https.proxy= push -u origin main
if ($LASTEXITCODE -ne 0) { throw "git push failed; run it manually to see the full error" }

Write-Step "done"
Write-Host "    https://github.com/$Owner/$Repo" -ForegroundColor Green
Write-Host ""
Write-Host "Next: Actions tab -> confirm CI is green, then docs/pre-release-checklist.md." -ForegroundColor Gray
