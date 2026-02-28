Import("env")

import os
import subprocess


def run_ota_bundle(source, target, env):
    project_dir = env.subst("$PROJECT_DIR")
    script_path = os.path.join(project_dir, "build_ota_bundle.sh")

    if not os.path.isfile(script_path):
        print("[OTA] build_ota_bundle.sh nicht gefunden, ueberspringe.")
        return

    process_env = os.environ.copy()
    process_env["VACUBEAR_POST_BUILD"] = "1"
    process_env["PIO_ENV"] = env.subst("$PIOENV")
    process_env["FW_BIN_PATH"] = env.subst("$BUILD_DIR/${PROGNAME}.bin")

    print("[OTA] Erzeuge dist/ota/latest via build_ota_bundle.sh ...")
    subprocess.check_call(["bash", script_path], cwd=project_dir, env=process_env)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", run_ota_bundle)
