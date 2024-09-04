from utils.config import Config
from gui import open_gui
from python_app_utils.log import Logger

def main():
    # App Initialization
    Config()
    logger = Logger()
    logger.init(Config)

    open_gui()

if __name__ == "__main__":
    main()