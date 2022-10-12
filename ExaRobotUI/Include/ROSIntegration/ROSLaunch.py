from launch import LaunchService 
import psutil
import os 
import sys
import importlib
import time
import signal
import threading
from multiprocessing import Process
from ament_index_python.packages import get_package_share_directory

class CROSLaunch():
    def __init__(self, astrPackageName, astrLaunchFile):
        super(CROSLaunch, self).__init__()
        self.launched = False
        try:
            self._launchPath = get_package_share_directory(astrPackageName)
            self._launchPath = os.path.join(self._launchPath, 'launch')
            # print('test ', self._launchPath)
        except:
            print("Can't find package: ", astrPackageName, self._launchPath)
        self._launchFile = astrLaunchFile
        self.ls = LaunchService()

        sys.path.extend([self._launchPath])
        sys.path = list(set(sys.path))

        try:
            self.launch_file = importlib.import_module(self._launchFile)
            self.ls.include_launch_description(self.launch_file.generate_launch_description())
            # print('test2 ',self.launch_file)
            self.proc_launch = Process(target=self.ls.run, name=astrLaunchFile)
        except:
            print("Can't import ", self._launchFile)

    def launch(self):
        self.proc_launch.start()
        self.launched = True
    
    def get_child_pid(self):
        if self.launched is False:
            return 
        
        current_process = psutil.Process()
        children = current_process.children(recursive=True)
        node = dict()
        i  = 0
        for child in children:
            # print('Child pid is {}:{}'.format(child.pid, psutil.Process(child.pid).name()))
            if i==1:
                node[psutil.Process(child.pid).name()] = child.pid    
            i = 1
        
        return node
    

    def terminate(self):
        if self.launched is False:
            return 
        current_process = psutil.Process()
        children = current_process.children(recursive=True)
        for child in children:
            child.send_signal(signal.SIGINT)          

if __name__ == '__main__':
    # test = CROSLaunch('urg_node', 'urg_node_launch')
    test = CROSLaunch('exa_robot','rs_multi_camera_launch')
    # test = CROSLaunch('ydlidar_ros2_driver', 'ydlidar_launch')
    # test = CROSLaunch('exa_robot','exa_nav_3d_launch')
    test.launch()
    # time.sleep(2)
    # print('test: ',test.get_child_pid())
    # print('Lunch test')
    # time.sleep(10)
    # print('Lunch terminate')
    # test.terminate()
    
    # test.proc_launch.terminate()
    
    try: 
        while 1:
            if test.proc_launch.is_alive() == True:
                time.sleep(1)
                # print('test: ',test.get_child_pid())
                node = test.get_child_pid()
                for name in node:
                    
                    print('{}:{}'.format(name, node[name]))
            else:
                print('exit')
                break
    except KeyboardInterrupt:
        print('Lunch terminate')
        test.terminate()
        
        while 1:
            time.sleep(1)
            print('exit')
            break
        
    

   
    