import matlab.engine

eng = matlab.engine.start_matlab()

eng.figureShow(nargout=0)

try:
    while True:
        a=1
except KeyboardInterrupt:
    pass