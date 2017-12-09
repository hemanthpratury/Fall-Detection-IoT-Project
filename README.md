# Fall-Detection-IoT-Project
There are several wearable devices in the market, which are used as activity trackers by fitness enthusiasts. Only a few devices/applications exist which provides features like
continuous health monitoring, notifying the concerned in case of medical emergency, assisting the user with voice service and aiding in the visualization of the health data.

The arduino code performs the calculations for the pitch, Fall and magnitude of the acceleration from the continuous accelerometer data.

Pitch: The rotation of the body about Y-axis in YZ plane is called as Pitch

Roll: The rotation of the body around X-axis in the XY plane.

Root Mean Square acceleration-
Acc_tot = Sqrt(ax^2 + ay^2 + az^2  )

Pitch	=	(acc_y/acc_tot) *57.296 	degrees
Roll	=	(acc_x/acc_tot) *-57.296 	degrees


Psuedo Code:
if Acc_tot > threshold:
  if ptch_angle > 50 or roll_angle > 50:
    send(Fall Alert!)
