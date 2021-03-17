from numpy import *

import importlib, argparse

import game

def main():
    parser = argparse.ArgumentParser(description = 'Physics engine demo.')
    parser.add_argument('definition', metavar = 'FILENAME', type = str, help = 'python file defining physics objects')
    parser.add_argument('-size', dest = 'size', type = int, default = [800, 600], nargs = 2, help = 'default size of the video')
    export = parser.add_argument_group('animation export arguments')
    export.add_argument('-export', action = 'store_true', default = False, help = 'export animation into PNG images')
    export.add_argument('-output', type = str, default = 'img', help = 'output path for PNG images')
    export.add_argument('-rate', type = int, default = 50, help = 'frame rate of the animation')
    export.add_argument('-duration', type = int, default = 5000, help = 'duration of the animation (ms)')
    args = parser.parse_args()
    
    module = importlib.import_module(args.definition)
    
    g = game.Game(area = module.Area, export = args.export, path = args.output, framerate = args.rate, duration = args.duration, size = args.size)
    g.elements = module.Elements
    g.start()

if __name__ == "__main__":
    main()
