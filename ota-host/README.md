# Free HTTPS OTA test host

Upload this folder to a new public GitHub repository, enable GitHub Pages for
the `main` branch and repository root, then use:

```text
https://YOUR_USERNAME.github.io/YOUR_REPOSITORY/ota/lte_lcd_project-3.1.5.bin
```

The included firmware is the ESP32-S3 LTE/LCD build with the LCD diagnostics
task started before modem startup.

SHA-256:

```text
00cf d8dc df3c 5c85 7354 2eac a4f6 13cc 4352 4690 2b7f e82a 7858 1925 a201 01ae
```

Keep this repository public only for testing; anyone who can access the URL
can download the firmware.
