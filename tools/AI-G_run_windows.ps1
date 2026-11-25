[CmdletBinding()]
param(
    [Parameter(Mandatory, Position=0)]
    [ValidateSet('Setup', 'Run', 'Frame', 'Restore', 'Deploy')]
    [string]$Action,
    [string]$Board = 'topst@192.168.0.100',
    [ValidateRange(1, 10000000)] [int]$Frames = 54000,
    [string]$Output = (Join-Path $env:USERPROFILE 'equilibrium_logs'),
    [string]$Binary,
    [string]$Compiler = 'aarch64-none-linux-gnu-gcc'
)
$ErrorActionPreference = 'Stop'

function Invoke-Checked($Program, [string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed (exit $LASTEXITCODE)." }
}

switch ($Action) {
    'Setup' {
        Invoke-Checked ssh @($Board, 'sh ~/cam.sh 90 && v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl exposure=300 && v4l2-ctl -d /dev/v4l-subdev2 --get-ctrl vertical_blanking,exposure,analogue_gain,digital_gain')
        Invoke-Checked ssh @('-tt', $Board, "su -c 'systemctl stop serial-getty@ttyAMA2.service && dmesg -n 1 && chmod 666 /dev/ttyAMA2'")
    }
    'Run' {
        Invoke-Checked ssh @('-tt', $Board, "exec ~/pend_cam --vflip --uart /dev/ttyAMA2 --baud 115200 --frames $Frames")
    }
    'Restore' {
        Invoke-Checked ssh @('-tt', $Board, "su -c 'systemctl start serial-getty@ttyAMA2.service'")
    }
    'Deploy' {
        $tmpBinary = Join-Path ([IO.Path]::GetTempPath()) ("pend_cam_" + [guid]::NewGuid().ToString('N'))
        try {
            if (!$Binary) {
                Invoke-Checked $Compiler @('-O2','-static','-o',$tmpBinary,(Join-Path $PSScriptRoot '..\aig\pend_cam.c'),'-lm')
                $Binary = $tmpBinary
            }
            $stage = 'pend_cam_' + [guid]::NewGuid().ToString('N')
            Invoke-Checked scp @($Binary, ("${Board}:" + $stage))
            Invoke-Checked ssh @($Board, "chmod 755 $stage && mv $stage pend_cam")
        } finally {
            if (Test-Path -LiteralPath $tmpBinary) { Remove-Item -LiteralPath $tmpBinary }
        }
    }
    'Frame' {
        $python = Join-Path $PSScriptRoot '..\.venv_windows\Scripts\python.exe'
        if (!(Test-Path -LiteralPath $python)) { $python = 'python' }
        $null = New-Item -ItemType Directory -Force -Path $Output
        $name = 'frame_' + [guid]::NewGuid().ToString('N')
        $raw = Join-Path $Output "$name.raw"
        $png = Join-Path $Output "$name.png"
        $remote = "/dev/shm/$name.raw"
        Invoke-Checked ssh @($Board, "sh ~/cam.sh view && v4l2-ctl -d /dev/video2 --stream-mmap --stream-count=1 --stream-to=$remote")
        try {
            Invoke-Checked scp @(("${Board}:" + $remote), $raw)
            Invoke-Checked $python @((Join-Path $PSScriptRoot 'frame_windows.py'), $raw, $png)
            Write-Output $png
        } finally {
            Invoke-Checked ssh @($Board, "rm -f $remote")
        }
    }
}
