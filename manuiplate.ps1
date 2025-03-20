# flatten_project.ps1
# Script to flatten project structure by moving all files to a single directory
# except those in 'tests' directories

# Create destination directory for all files
$destination = Join-Path -Path (Get-Location) -ChildPath "flattened_project"
New-Item -ItemType Directory -Path $destination -Force | Out-Null
Write-Host "Created destination directory: $destination"

# Get all files recursively from current directory
$allFiles = Get-ChildItem -Path (Get-Location) -Recurse -File

# Process each file
foreach ($file in $allFiles) {
    # Skip files that are in tests directories or already in the destination
    $isInTestsDir = $file.FullName -match "\\tests\\" -or $file.DirectoryName -match "\\tests\\"
    $isInDestination = $file.DirectoryName -eq $destination
    
    if (-not $isInTestsDir -and -not $isInDestination) {
        # Check if file with same name already exists in destination
        $destFile = Join-Path -Path $destination -ChildPath $file.Name
        if (Test-Path -Path $destFile) {
            Write-Host "Warning: File already exists in destination: $($file.Name). Renaming..."
            # Create a unique name by adding directory info
            $parentDir = Split-Path -Leaf (Split-Path -Parent $file.FullName)
            $newName = "{0}_{1}" -f $parentDir, $file.Name
            $destFile = Join-Path -Path $destination -ChildPath $newName
        }
        
        # Copy file to destination
        Copy-Item -Path $file.FullName -Destination $destFile
        Write-Host "Copied: $($file.FullName) -> $destFile"
    }
}

Write-Host "Project flattening complete. Files are now in: $destination"
Write-Host "Note: Original files have not been deleted. You can delete them manually if needed."