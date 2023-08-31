mc_rtc new plugin template
==

This project is a template for a new plugin wihtin [mc_rtc]

It comes with:
- a CMake project that can build a plugin for [mc_rtc], the project can be put within [mc_rtc] source-tree for easier updates
- clang-format files
- automated GitHub Actions builds on three major platforms

Quick start
--

1. Renaming the controller from `PhidgetPressureSensorPlugin` to `PhidgetPressureSensorPlugin`. In a shell (Git Bash on Windows, replace sed with gsed on macOS):

```bash
sed -i -e's/PhidgetPressureSensorPlugin/PhidgetPressureSensorPlugin/g' `find . -type f`
git mv src/PhidgetPressureSensorPlugin.cpp src/PhidgetPressureSensorPlugin.cpp
git mv src/PhidgetPressureSensorPlugin.h src/PhidgetPressureSensorPlugin.h
git mv etc/PhidgetPressureSensorPlugin.in.yaml etc/PhidgetPressureSensorPlugin.in.yaml
```

2. You can customize the project name in vcpkg.json as well, note that this must follow [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md)

3. Build and install the project

4. Add the following udev rule to allow running as non-root: edit `/etc/udev/rules.d/51-usb-device.rules`
  ```sh
  SUBSYSTEM=="usb", MODE="0660", GROUP="plugdev"
  ```

4. Run using your [mc_rtc] interface of choice, add `PhidgetPressureSensorPlugin` to the `Plugins` configuration entry or enable the autoload option

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
