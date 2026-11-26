import os
import subprocess
import sys

tools_make = r"E:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
tools_gcc  = r"E:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin"

env = os.environ.copy()
env["PATH"] = f"{tools_make};{tools_gcc};" + env.get("PATH", "")

make_bin = os.path.join(tools_make, "make.exe")
debug_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Debug")

print("Building firmware with make in:", debug_dir)
cmd = [make_bin, "-j8", "all"]
res = subprocess.run(cmd, cwd=debug_dir, env=env, capture_output=True, text=True)

print("STDOUT:\n", res.stdout)
if res.stderr:
    print("STDERR:\n", res.stderr)
print("Exit code:", res.returncode)
sys.exit(res.returncode)
