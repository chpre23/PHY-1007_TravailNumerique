
import numpy as np, math
from scipy.constants import mu_0
from fields import*


def relax(A, maxsteps, convergence):
    """
    Relaxes the matrix A until the sum of the absolute differences
    between the previous step and the next step (divided by the number of
    elements in A) is below convergence, or maxsteps is reached.

    Input:
    - A: matrix to relax
    - maxsteps, convergence: Convergence criterions

    Output:
    - A is relaxed when this method returns
    """

    iterations = 0
    diff = convergence +1

    Nx = A.shape[1]
    Ny = A.shape[0]
    
    while iterations < maxsteps and diff > convergence:
        #Loop over all *INNER* points and relax
        Atemp = A.copy()
        diff = 0.0
        
        for y in range(1,Ny-1):
            for x in range(1,Ny-1):
                A[y,x] = 0.25*(Atemp[y,x+1]+Atemp[y,x-1]+Atemp[y+1,x]+Atemp[y-1,x])
                diff  += math.fabs(A[y,x] - Atemp[y,x])

        diff /=(Nx*Ny)
        iterations += 1
        print("Iteration #", iterations, ", diff =", diff)


def boundary(A,x,y):
    """
    Set up boundary conditions

    Input:
    - A: Matrix to set boundaries on
    - x: Array where x[i] = hx*i, x[last_element] = Lx
    - y: Eqivalent array for y

    Output:
    - A is initialized in-place (when this method returns)
    """

    #Boundaries implemented (condensator with plates at y={0,Lx}, DeltaV = 200):
    # A(x,0)  =  100*sin(2*pi*x/Lx)
    # A(x,Ly) = -100*sin(2*pi*x/Lx)
    # A(0,y)  = 0
    # A(Lx,y) = 0

    Nx = A.shape[1]
    Ny = A.shape[0]
    Lx = x[Nx-1] #They *SHOULD* have same sizes!
    Ly = x[Nx-1]


    A[:,0]    =   100*numpy.sin(math.pi*x/Lx)
    A[:,Nx-1] = - 100*numpy.sin(math.pi*x/Lx)
    A[0,:]    = 0.0
    A[Ny-1,:] = 0.0


#Main program

import sys

# Input parameters
Nx      = 100
Ny      = 100
maxiter = 1000

x = numpy.linspace(0,1,num=Nx+2) #Also include edges
y = numpy.linspace(0,1,num=Ny+2)
A = numpy.zeros((Nx+2,Ny+2))
numpy.set_printoptions(threshold=numpy.inf)
#print(A.shape)

boundary(A,x,y)
#Remember: as solution "creeps" in from the edges,
#number of steps MUST AT LEAST be equal to
#number of inner meshpoints/2 (unless you have a better
#estimate for the solution than zeros() )
relax(A,maxiter,0.00001)


def biot_savart(I, dx, dy):
    """
    Calculate the magnetic field using Biot-Savart law.
    
    Parameters:
        I (np.ndarray): Matrix of currents.
        dx (float): Distance between points in the x-direction.
        dy (float): Distance between points in the y-direction.
        
    Returns:
        np.ndarray: Magnetic field matrix.
    """
    nx, ny = I.shape
    B = np.zeros((2, nx, ny))
    
    for i in range(nx):
        for j in range(ny):
            for k in range(nx):
                for l in range(ny):
                    if i == k and j == l:
                        continue
                    
                    r = np.array([dx * (i - k), dy * (j - l), 0])
                    r_norm = np.linalg.norm(r)
                    dB = mu_0 / (4 * np.pi) * I[k, l] / r_norm ** 3 * np.cross(np.array([0, 0, 1]), r)
                    B[i, j] += dB
    
    return B[:, :, 2]

# Example usage
I = np.array([[1, 0, -1], [2, 0, -2], [1, 0, -1]])
dx = 0.1
dy = 0.1
B = biot_savart(I, dx, dy)
print(I)

print(np.zeros(3))
B = np.zeros((2, 4, 3))
print(len(B[1]))

r = np.array([dx * (3 - 1), dy * (3 - 1), 0])
print(np.cross(np.array([0, 0, 1]), r))

#._circuit.get_voltage_and_current_fields(self._shape, self.minimum, self.maximum)
#print(numpy.ndarray((2, 2, 2)))
##################### TESTS BIO-SAVART CARTESIEN #######################
import numpy as np, math
from scipy.constants import mu_0
from fields import*


#field1 = np.random.rand(20, 20, 3)
#print(Vectorfield1[1, 1])
field1 = np.random.rand(100, 100, 3)

#print(field1)

#Vectorfield1 = VectorField(field1)
#Vectorfield1 = VectorField(field1)

#print('TOTO : ', field1[50, 50])

#Vectorfield1.show()
#Vectorfield2.show()

def compute_biot_savart(matrix, point):
    magnetic_field = np.zeros(3)  # Initialize the magnetic field vector
    dim_i = matrix.shape[0]
    dim_j = matrix.shape[1]
    for i in range(dim_i):
        #print(len(matrix))
        for j in range(dim_j):
            #print(len(matrix[i]))
            current_element = matrix[i][j]
            position_vector = np.array([i, j, 0])  # Assuming a 2D vector field with z-component as 0
            
            # Calculate the distance between the current element and the given point
            distance = np.linalg.norm(point - position_vector)
            
            if distance != 0 :
            # Calculate the contribution to the magnetic field using the Biot-Savart law
                magnetic_field += np.cross(current_element, (point - position_vector)) / distance**3
            else:
                continue      
    return magnetic_field

#point = np.array([1, 1, 0])


#electric_current[2][3] = [0, 0, 0]
#print(electric_current[2][3].any())
#if electric_current[2][3].all() == 3:
#    print('yep')

electric_current = VectorField(field1)
#electric_current.show()
magnetic_array = np.zeros((100, 100, 3))

#electric_current[1][2] = [1, 0, 0]
#print(electric_current[1][2])
#print(electric_current[1][2].any())
#print(len(electric_current))
#print(len(electric_current[x]))

#field1 = np.random.rand(5, 3, 3)
#print(np.zeros((5, 3, 3)))
#print(field1)
#electric_current = VectorField(field1)
#print(len(electric_current.x[0]))
#print(electric_current.shape[0])
#print(len(electric_current[1]))


#magnetic_array = np.zeros((20, 20, 3))
n = 0
dim_x = electric_current.shape[0]
dim_y = electric_current.shape[1]
for x in range(dim_x):
    for y in range(dim_y):
        if electric_current[x][y].any() == True:
            resultat = compute_biot_savart(electric_current, np.array([x, y, 0]))
            magnetic_array[x, y] = resultat
            print('nb iterations : ', n)
            n += 1

Vectorfield_mag = VectorField(magnetic_array)
Vectorfield_mag.show()



import numpy as np, math
from scipy.constants import mu_0
from fields import*

field1 = np.random.rand(150, 150, 3)
electric_current = VectorField(field1)
#magnetic_array = np.zeros((150, 150, 3))

def compute_biot_savart(matrix, point, delta_x, delta_y):
    # Calculate the position vectors for all elements in the matrix
    position_vectors = np.array([[i*delta_x, j*delta_y, 0] for i in range(len(matrix)) for j in range(len(matrix[i]))])
    
    # Calculate the distances between the position vectors and the given point
    distances = np.linalg.norm(point - position_vectors, axis=1)
    distances_3d = np.repeat(distances[:, np.newaxis], 3, axis=1)
    a = np.cross(matrix.reshape((-1, 3)), (point - position_vectors))
    # Calculate the magnetic field contribution using the Biot-Savart law
    magnetic_field = np.sum(np.divide(a, distances_3d**3, out=np.zeros_like(a), where=distances_3d!=0), axis=0)
    #print(magnetic_field)
    return magnetic_field

def optimize_compute_biot_savart(matrix, delta_x, delta_y):
    non_zero_indices = np.argwhere(matrix.any(axis=2))  # Get indices where the matrix is non-zero
    print(non_zero_indices)
    magnetic_array = np.zeros((matrix.shape[0], matrix.shape[1], 3))
    n = 0
    for x, y in non_zero_indices:
        magnetic_array[x, y] = compute_biot_savart(matrix, np.array([x*delta_x, y*delta_y, 0]), delta_x, delta_y)
        print('nb iterations:', n)
        n += 1

    Vectorfield_mag = VectorField(magnetic_array)
    return Vectorfield_mag

optimize_compute_biot_savart(electric_current, 1, 1)

#Vectorfield_mag.show()























import numpy as np, math
from scipy.constants import mu_0
from fields import*

def biot_savart_law(current_matrix, dx, dy):
    # Get the shape of the current matrix
    shape = current_matrix.shape
    
    # Create arrays to store the magnetic field components
    Bx = np.zeros(shape)
    By = np.zeros(shape)
    Bz = np.zeros(shape)
    
    # Get the indices of non-zero elements in the current matrix
    indices = np.nonzero(current_matrix)
    
    # Iterate over the non-zero elements
    for i in range(len(indices[0])):
        # Get the coordinates of the current element
        x, y, z = indices[0][i], indices[1][i], indices[2][i]
        
        # Iterate over all other elements to calculate the magnetic field
        for j in range(shape[0]):
            for k in range(shape[1]):
                for l in range(shape[2]):
                    # Calculate the distance between the current element and the other element
                    r = np.sqrt((x-j)**2 + (y-k)**2 + (z-l)**2)
                    
                    # Calculate the magnetic field components using the Biot-Savart law
                    Bx[j, k, l] += (current_matrix[x, y, z] * (z-l)) / (4 * np.pi * r**3)
                    By[j, k, l] += (current_matrix[x, y, z] * (l-z)) / (4 * np.pi * r**3)
                    Bz[j, k, l] += (current_matrix[x, y, z] * (x-j)) / (4 * np.pi * r**3)
    
    # Scale the magnetic field components by dx and dy
    Bx *= dx
    By *= dy
    Bz *= dx * dy
    
    # Return the magnetic field components as a 3D matrix
    return np.stack((Bx, By, Bz), axis=3)

field1 = np.random.rand(20, 20, 3)
electric_current = VectorField(field1)
biot_savart_law(electric_current, 1, 1)

############# TESTS BIO-SAVART POLAIRE ################
# Voir les valeurs contenues dans la matrice de depart
import numpy as np, math
from scipy.constants import mu_0
from fields import*

mu_0 = 4 * np.pi * 1e-7  # Permeability of free space
delta_r = ...  # Radial increment
delta_theta = ...  # Angular increment

def biot_savart(current, position, segment):
    r = position - segment  # Vector from the segment to the position
    r_norm = np.linalg.norm(r)
    segment_norm = np.linalg.norm(segment)
    cross_product = np.cross(segment, r)
    return (mu_0 / (4 * np.pi)) * (current * cross_product) / (r_norm ** 3)

def apply_biot_savart_law(vector_field):
    dimensions = vector_field.shape
    total_magnetic_field = np.zeros(dimensions)
    for i in range(dimensions[0]):
        for j in range(dimensions[1]):
            for k in range(dimensions[2]):
                position = np.array([i * delta_r, j * delta_theta, k * delta_theta])
                for segment in vector_field:
                    current = segment.current  # Extract the current value from the segment
                    magnetic_field = biot_savart(current, position, segment.position)
                    total_magnetic_field[i, j, k] += magnetic_field
    return total_magnetic_field


