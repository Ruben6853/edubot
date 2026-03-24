# run_hw.ps1 – Koppelt USB-robot aan WSL en runt de hardware demo
# Dubbelklik of: Right-click → "Run with PowerShell"
# Vereist: usbipd  (winget install usbipd)

# ── Zelf opnieuw starten als Administrator ────────────────────────────────────
if (-NOT ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]"Administrator")) {
    Write-Host "Herstart als Administrator..." -ForegroundColor Yellow
    Start-Process powershell "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`"" -Verb RunAs
    exit
}

Write-Host ""
Write-Host "=== EduBot Hardware Demo ===" -ForegroundColor Cyan
Write-Host ""

# ── Controleer of usbipd beschikbaar is ──────────────────────────────────────
if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    Write-Host "[FOUT] usbipd niet gevonden." -ForegroundColor Red
    Write-Host "Installeer via:  winget install usbipd" -ForegroundColor Yellow
    Read-Host "Druk Enter om te sluiten"
    exit 1
}

# ── Laat USB-apparaten zien ───────────────────────────────────────────────────
Write-Host "Beschikbare USB-apparaten:" -ForegroundColor Green
usbipd list
Write-Host ""

# ── Zoek automatisch naar bekende robot USB-adapters ─────────────────────────
$robotDevice = usbipd list | Select-String -Pattern "CH340|CP210|FTDI|USB Serial|1a86:7523|10c4:ea60|0403:6001" | Select-Object -First 1

if ($robotDevice) {
    $busid = ($robotDevice.Line -split "\s+")[0]
    Write-Host "Robot gevonden: $($robotDevice.Line)" -ForegroundColor Green
    Write-Host "BUSID: $busid" -ForegroundColor Green
} else {
    Write-Host "Robot niet automatisch herkend." -ForegroundColor Yellow
    $busid = Read-Host "Voer de BUSID in (bijv. 1-3)"
}

Write-Host ""

# ── Bind en koppel aan WSL ────────────────────────────────────────────────────
Write-Host "Koppelen aan WSL..." -ForegroundColor Green
try { usbipd bind --busid $busid --force 2>$null } catch {}

usbipd attach --wsl --busid $busid
if ($LASTEXITCODE -ne 0) {
    Write-Host "[FOUT] USB koppelen mislukt. Controleer of WSL (Ubuntu) draait." -ForegroundColor Red
    Read-Host "Druk Enter om te sluiten"
    exit 1
}
Write-Host "USB succesvol gekoppeld aan WSL!" -ForegroundColor Green
Write-Host ""

# ── Controleer /dev/ttyUSB0 in WSL ───────────────────────────────────────────
Write-Host "Controleren USB in WSL..." -ForegroundColor Green
$tty = wsl.exe -d Ubuntu bash -c "ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo 'GEEN_TTY'"
if ($tty -match "GEEN_TTY") {
    Write-Host "[WARN] Geen /dev/ttyUSB* gevonden in WSL." -ForegroundColor Yellow
    Write-Host "Probeer: wsl --shutdown en opnieuw proberen." -ForegroundColor Yellow
} else {
    Write-Host "Gevonden: $tty" -ForegroundColor Green
    wsl.exe -d Ubuntu bash -c "sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true"
}

Write-Host ""
Write-Host "Starten hardware demo..." -ForegroundColor Cyan
Write-Host "(Zorg dat de robot aan staat)" -ForegroundColor Yellow
Write-Host ""

# ── Draai de demo in WSL ──────────────────────────────────────────────────────
wsl.exe -d Ubuntu bash /mnt/c/Users/Wesse/OneDrive/Documents/GitHub/edubot/Wessel/run_demo.sh hw

# ── Ontkoppel USB na afloop ───────────────────────────────────────────────────
Write-Host ""
Write-Host "Demo klaar. USB loskoppelen van WSL..." -ForegroundColor Green
usbipd detach --busid $busid 2>$null

Read-Host "Druk Enter om te sluiten"
