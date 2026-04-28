$port = New-Object System.IO.Ports.SerialPort COM10,115200,None,8,One
$port.DtrEnable = $false
$port.RtsEnable = $true
$port.Open()
Start-Sleep -Milliseconds 100
$port.DtrEnable = $true
$port.RtsEnable = $false
Start-Sleep -Milliseconds 100
$port.RtsEnable = $true
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.ElapsedMilliseconds -lt 8000) {
    if ($port.BytesToRead -gt 0) {
        Write-Host $port.ReadLine()
    }
}
$port.Close()
