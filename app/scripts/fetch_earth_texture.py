#!/usr/bin/env python3
"""
Download a NASA Blue Marble Earth texture for the visualizer.

Usage:
    python3 scripts/fetch_earth_texture.py

This downloads a 2048x1024 equirectangular Earth daylight map from NASA's
Visible Earth collection and saves it to res/earth_daymap.jpg.

If the download fails (firewall, etc.), the application will fall back
to a solid blue sphere.

Alternative: manually download any equirectangular Earth texture and
place it at res/earth_daymap.jpg (or .png).

NASA source (public domain):
    https://visibleearth.nasa.gov/images/73909/december-blue-marble-next-generation-w-topography-and-bathymetry
"""

import os
import sys
import urllib.request

# 2k resolution — good balance of quality and file size (~700 KB).
# For higher res, use the 8k version from NASA Visible Earth.
URL = "https://eoimages.gsfc.nasa.gov/images/imagerecords/73000/73909/world.topo.bathy.200412.3x5400x2700.jpg"
# Fallback: smaller version
URL_FALLBACK = "https://eoimages.gsfc.nasa.gov/images/imagerecords/73000/73909/world.topo.bathy.200412.3x2048x1024.jpg"

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    res_dir = os.path.join(os.path.dirname(script_dir), "res")
    os.makedirs(res_dir, exist_ok=True)

    output = os.path.join(res_dir, "earth_daymap.jpg")

    if os.path.exists(output):
        print(f"Earth texture already exists: {output}")
        print("Delete it and re-run to re-download.")
        return

    for url in [URL_FALLBACK, URL]:
        print(f"Downloading Earth texture from NASA...")
        print(f"  URL: {url}")
        try:
            urllib.request.urlretrieve(url, output)
            size_mb = os.path.getsize(output) / (1024 * 1024)
            print(f"  Saved: {output} ({size_mb:.1f} MB)")
            return
        except Exception as e:
            print(f"  Failed: {e}")
            if os.path.exists(output):
                os.remove(output)
            continue

    print("\nCould not download Earth texture.")
    print("The application will use a solid blue sphere as fallback.")
    print("You can manually place an equirectangular Earth texture at:")
    print(f"  {output}")

if __name__ == "__main__":
    main()