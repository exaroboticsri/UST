rm -rf docs/source/Inc docs/source/Lib
sphinx-apidoc -f -o docs/source/Inc/AudioRecognition ./Include/AudioRecognition
sphinx-apidoc -f -o docs/source/Inc/SpeechSynthesis ./Include/SpeechSynthesis
sphinx-apidoc -f -o docs/source/Inc/Dialog ./Include/Dialog
sphinx-apidoc -f -o docs/source/Inc/DockWidgets ./Include/DockWidgets
sphinx-apidoc -f -o docs/source/Inc/ExternalComm ./Include/ExternalComm
sphinx-apidoc -f -o docs/source/Inc/MdiBackground ./Include/MdiBackground
sphinx-apidoc -f -o docs/source/Inc/Misc ./Include/Misc
sphinx-apidoc -f -o docs/source/Inc/MobileRobot ./Include/MobileRobot
sphinx-apidoc -f -o docs/source/Inc/MultiProcessing ./Include/MultiProcessing
sphinx-apidoc -f -o docs/source/Inc/PathPlanning ./Include/PathPlanning
sphinx-apidoc -f -o docs/source/Inc/ROSIntegration ./Include/ROSIntegration
sphinx-apidoc -f -o docs/source/Inc/SubWindow ./Include/SubWindow
sphinx-apidoc -f -o docs/source/Inc/SubWindow/SerialTerm ./Include/SubWindow/SerialTerm
sphinx-apidoc -f -o docs/source/Inc/TrajectoryGenerator ./Include/TrajectoryGenerator
sphinx-apidoc -f -o docs/source/Inc/ ./Include/


sphinx-apidoc -f -o docs/source/Lib/Devices/Camera ./Library/Devices/Camera
sphinx-apidoc -f -o docs/source/Lib/Serial ./Library/Serial
sphinx-apidoc -f -o docs/source/Lib/Socket ./Library/Socket




