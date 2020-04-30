#!/usr/bin/env python
import os
import sys
import pickle
from subprocess import Popen

from config import Nav_Pickle_File
from utils.logger import logger

if __name__ == "__main__":
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "robotmaster.settings")

    #kill existed navigation process
    logger.info('[manage.py] try to kill existed navigation process!')
    if os.path.exists(Nav_Pickle_File):
        try:
            with open(Nav_Pickle_File, 'rb') as f:
                proc = pickle.load(f)
                proc.terminate()
        except OSError as e:
            logger.info(str(e))
        os.remove(Nav_Pickle_File)

    try:
        from django.core.management import execute_from_command_line
    except ImportError:
        # The above import may fail for some other reason. Ensure that the
        # issue is really that Django is missing to avoid masking other
        # exceptions on Python 2.
        try:
            import django
        except ImportError:
            raise ImportError(
                "Couldn't import Django. Are you sure it's installed and "
                "available on your PYTHONPATH environment variable? Did you "
                "forget to activate a virtual environment?"
            )
        raise
    execute_from_command_line(sys.argv)
