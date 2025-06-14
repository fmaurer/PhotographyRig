import subprocess

def get_outdated_packages():
    result = subprocess.run(
        ["pip", "list", "--outdated", "--format=freeze"],
        capture_output=True,
        text=True,
    )
    packages = []
    for line in result.stdout.strip().split("\n"):
        if "==" in line:
            pkg = line.split("==")[0]
            packages.append(pkg)
    return packages

def upgrade_packages(packages, use_break=False):
    for pkg in packages:
        print(f"Upgrading {pkg}...")
        cmd = ["pip", "install", "--upgrade", pkg]
        if use_break:
            cmd.append("--break-system-packages")
        subprocess.run(cmd)

if __name__ == "__main__":
    pkgs = get_outdated_packages()
    if not pkgs:
        print("✅ All packages are up to date.")
    else:
        print(f"📦 Packages to upgrade: {pkgs}")
        upgrade_packages(pkgs, use_break=True)  # Set to False if you're in a virtualenv
