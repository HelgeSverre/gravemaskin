core := "src/Gravemaskin.Core/Gravemaskin.Core.fsproj"
tests := "tests/Gravemaskin.Tests/Gravemaskin.Tests.fsproj"
solution := "Gravemaskin.slnx"
dotnet := "PATH=\"$PWD/.dotnet:$PATH\" dotnet"

[private]
default:
    @just --list

# Ensure a compatible SDK is available (install into .dotnet/ if needed).
[private]
[unix]
_sdk:
    #!/usr/bin/env bash
    set -euo pipefail
    if PATH="$PWD/.dotnet:$PATH" dotnet --version >/dev/null 2>&1; then exit 0; fi
    echo "No SDK satisfying global.json found — installing into .dotnet/"
    installer="${TMPDIR:-/tmp}/gravemaskin-dotnet-install.sh"
    curl -fsSL https://dot.net/v1/dotnet-install.sh -o "$installer"
    bash "$installer" --jsonfile global.json --install-dir "$PWD/.dotnet"

[private]
[windows]
_sdk:
    @dotnet --version >nul 2>&1 || echo "No SDK satisfying global.json. Install it with dotnet-install.ps1."

# Restore dependencies.
[group('build')]
restore: _sdk
    {{ dotnet }} restore {{ solution }}

# Build solution.
[group('build')]
build: _sdk
    {{ dotnet }} build {{ solution }}

# Build solution (Release).
[group('build')]
release-build: _sdk
    {{ dotnet }} build {{ solution }} -c Release

# Clean build output.
[group('build')]
clean: _sdk
    {{ dotnet }} clean {{ solution }}

# Format code.
[group('format')]
format: _sdk
    {{ dotnet }} format {{ solution }} --no-restore

# Check formatting.
[group('format')]
lint: _sdk
    {{ dotnet }} format {{ solution }} --verify-no-changes --no-restore

# Run fast tests.
[group('test')]
test: _sdk
    {{ dotnet }} test {{ tests }} --nologo --filter "Category!=Integration"

# Run slow integration scenarios (long headless sims).
[group('test')]
smoke: _sdk
    {{ dotnet }} test {{ tests }} --nologo --filter "Category=Integration"

# Pre-commit gate.
[group('test')]
check: lint build test smoke

# Release-build perf gate: the strict tick-budget numbers.
[group('test')]
perf: _sdk
    {{ dotnet }} test {{ tests }} -c Release --nologo --filter "FullyQualifiedName~budget"
