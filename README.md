# KELO Tulip

This package contains the *KELO Tulip* software.

## Permissions

Special permissions need to be granted to the executable, since it needs *RAW* access to the Ethernet port. Instead of starting it
as root the `setcap` tool can be used:
 
```
sudo setcap cap_net_raw+ep <name_of_executable>

```

Optionally, the command can be applied during the build process by passing the `-DUSE_SETCAP=ON` option to catkin. Default is `OFF`.

```
catkin_make -DUSE_SETCAP=ON
```
OR
```
catkin build kelo_tulip -DUSE_SETCAP=ON
```

**Note**: If you use this flag, you will be asked to input sudo password during
the build process. It would look like the build is going on but you need to keep
a lookout for `[sudo] password for <username>` in the output and enter the
password as soon as this prompt is visible. If not, the build process will
continue forever.



