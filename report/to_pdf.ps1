# 説明資料 docx を Word で PDF に変換する
#   powershell -File to_pdf.ps1
$out = Join-Path $PSScriptRoot "..\output"
$docx = (Resolve-Path (Join-Path $out "説明資料_PAI最終課題.docx")).Path
$pdf = Join-Path (Resolve-Path $out).Path "説明資料_PAI最終課題_アカウント名.pdf"
$w = New-Object -ComObject Word.Application
$w.Visible = $false
$w.DisplayAlerts = 0
$d = $w.Documents.Open($docx, $false, $true)
$d.ExportAsFixedFormat($pdf, 17)
"pages: " + $d.ComputeStatistics(2)
$d.Close($false)
$w.Quit()
"saved: $pdf"
