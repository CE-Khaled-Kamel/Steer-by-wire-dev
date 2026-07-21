import matplotlib.pyplot as plt
import numpy as np

# Pacejka Magic Tire Formula
# Calulates the Lateral Force(y) on a tire given Slip Angle(x)

# Empirical Coeffeients
D = 4030.34  # The Peak Factor: controls max lateral acceleration before breaking traction
C = 1.3  # The Shape Factor: contols curve asymptote
B = 9.54 # The Stiffness Factor: controls how much lateral force generated in responce to steering input
E = 0.1 # The Curvature Factor: controls how fast the tire loses traction after curve peak

# Generate a range of Slips Angles
xDeg = np.linspace(-180,180,360)
# Convert from degrees to radinas
x = np.radians(xDeg)

y = D * np.sin( C * np.arctan( B*x - E*( B*x - np.arctan(B*x) ) ) )

plt.figure(figsize=(8,5))
plt.plot(xDeg, y, color="blue", linewidth=3, label="Magic Curve")

plt.title("Pecejka Magic Formula Curve for D="+str(D)+", C="+str(C)+", B="+str(B)+", E="+str(E))
plt.xlabel("Slip Angle(x)")
plt.ylabel("Lateral force(y)")
plt.grid(True)
plt.show()