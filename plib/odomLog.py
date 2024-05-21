#!/usr/bin/python3
#
# odomLog.py   allow user scripts to 
#              log to odometer.log
#
# Formats message as:
# YYYY-MM-DD HH:MM|[<script.py>.<funcName>]<message>
#
# USAGE:
#    import sys
#    sys.path.append('/home/pi/HOME/plib')
#    import odomLog
#
#
#    def somefunc():
#        strToLog = "*** Reset Encoders ***"
#        odomLog.logger.info(strToLog)  or
#        odomLog.logger.info("message") 
#

from __future__ import print_function

import sys
import logging

HOME="GoPi5Go"
# create logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

loghandler = logging.FileHandler('/home/pi/'+HOME+'/logs/odometer.log')

logformatter = logging.Formatter('%(asctime)s|[%(filename)s.%(funcName)s]%(message)s',"%Y-%m-%d %H:%M")
loghandler.setFormatter(logformatter)
logger.addHandler(loghandler)



def testfunc():
    strToLog = "*** odomLog.py testfunc() executed ***"
    logger.info(strToLog)
    print("odomLog.py testfunc() logged:",strToLog)

# test main 
def main():

    testfunc()


if __name__ == "__main__":
    main()




