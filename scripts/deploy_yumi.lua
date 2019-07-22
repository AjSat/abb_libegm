-- Creating Orocos component for ABB Yumi robot


require("rttlib")
require("rttros")
rtt.setLogLevel("Warning")
rttlib.color = true

gs=rtt.provides()
tc=rtt.getTC()

if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end

depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("rtt_rospack")

depl:import("abb_libegm")