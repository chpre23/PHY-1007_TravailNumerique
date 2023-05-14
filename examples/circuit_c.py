import env_examples  # Modifies path, DO NOT REMOVE
from sympy import sqrt
from sympy import Symbol
from src import Circuit, CoordinateSystem, VoltageSource, Wire, World


if __name__ == "__main__":
    # INITIALISATION DES PARAMETRES DE BASE
    WORLD_SHAPE = (150, 150)
    BATTERY_VOLTAGE = 1.0
    HIGH_WIRE_RESISTANCE = 1.0
    LOW_WIRE_RESISTANCE = 0.01

    # INITIALISATION DES VARIABLES
    cartesian_variables = Symbol("x"), Symbol("y")
    x, y = cartesian_variables

    # INITIALISATION DES EQUATIONS PARAMETRIQUES
    x_exp_d = sqrt(2500 - (y + 50)**2)
    y_exp_d = y
    half_circle_d = (x_exp_d, y_exp_d)

    x_exp_g = -sqrt(2500 - (y - 50)**2)
    y_exp_g = y
    half_circle_g = (x_exp_g, y_exp_g)
    x_exp_s = x
    y_exp_s = 0 * y
    s_exp = (x_exp_s, y_exp_s)
    
    # CREATION DES PARTIES DU CIRCUIT
    wires = [
        VoltageSource((70, 25), (80, 25), s_exp, cartesian_variables, BATTERY_VOLTAGE),         # SOURCE TENSION
        Wire((70, 25), (70, 125), half_circle_g, cartesian_variables, LOW_WIRE_RESISTANCE),     # FIL BAS GAUCHE
        Wire((70, 125), (80, 125), s_exp, cartesian_variables, HIGH_WIRE_RESISTANCE),           # FIL HAUT
        Wire((80, 125), (80, 25), half_circle_d, cartesian_variables, LOW_WIRE_RESISTANCE)      # FIL BAS DROIT
    ]
    # GROUND
    ground_position = (70, 25)
    
    # CREATION DU CIRCUIT ET DE LA MAP
    circuit = Circuit(wires, ground_position)
    world = World(circuit=circuit, coordinate_system=CoordinateSystem.CARTESIAN, shape=WORLD_SHAPE)
    
    # AFFICHAGE DU CIRCUIT
    world.show_circuit(
        #   Met des nodes sur les points d'intersection
        {0: (70, 25), 1:(80, 25), 2: (70, 125), 3: (80, 125)}
    )
    
    # CALCULS DES CHAMPS
    world.compute()
    
    # AFFICHAGE DES CHAMPS
    world.show_all()