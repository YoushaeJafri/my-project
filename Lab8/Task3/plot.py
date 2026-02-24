import serial
import matplotlib.pyplot as plt

gyroData = serial.Serial('/dev/ttyACM0',115200)

plt.ion()

x_dps_values=[]
y_dps_values=[]
z_dps_values=[]
temp_values=[]
time_ms=[]
cnt=0

fig,(ax1,ax2)=plt.subplots(2,1)

# Create lines ONCE
line_x,=ax1.plot([],[],'r-',label='X')
line_y,=ax1.plot([],[],'g-',label='Y')
line_z,=ax1.plot([],[],'b-',label='Z')
line_t,=ax2.plot([],[],'m-',label='Temp')

ax1.set_ylim(-300,300)
ax2.set_ylim(0,60)
ax1.legend()
ax2.legend()

while True:

    line=gyroData.readline().decode().strip()
    values=line.split(',')

    if len(values)==4:

        x=float(values[1])
        y=float(values[2])
        z=float(values[3])
        t=float(values[0])

        x_dps_values.append(x)
        y_dps_values.append(y)
        z_dps_values.append(z)
        temp_values.append(t)
        time_ms.append(cnt)
        cnt+=100

        # keep last 500
        if len(x_dps_values)>500:
            x_dps_values.pop(0)
            y_dps_values.pop(0)
            z_dps_values.pop(0)
            temp_values.pop(0)
            time_ms.pop(0)

        # UPDATE ONLY DATA (FAST)
        line_x.set_data(time_ms,x_dps_values)
        line_y.set_data(time_ms,y_dps_values)
        line_z.set_data(time_ms,z_dps_values)
        line_t.set_data(time_ms,temp_values)

        ax1.set_xlim(time_ms[0],time_ms[-1])
        ax2.set_xlim(time_ms[0],time_ms[-1])

        plt.pause(0.001)