import rpyc

from controller import *

class RobotService(rpyc.Service):
    def on_connect(self, conn):
        # code that runs when a connection is created
        # (to init the service, if needed)
        pass

    def on_disconnect(self, conn):
        # code that runs after the connection has already closed
        # (to finalize the service, if needed)
        pass
        
    exposed_robot = Supervisor()
    
if __name__ == "__main__":
    from rpyc.utils.server import ThreadedServer
    t = ThreadedServer(RobotService(), port=18861, protocol_config={'allow_public_attrs': True,})
    t.start()