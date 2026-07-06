import os
from .processing_Video import SSGVision

if __name__ == "__main__":
    config_path = "config.yaml"
    if not os.path.exists(config_path):
        raise Exception(f"Config file {config_path} does not exist.")
    vision_system = SSGVision(config_path)
    # vision_system.run()