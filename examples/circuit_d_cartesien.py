import env_examples  # Modifies path, DO NOT REMOVE
from sympy import Symbol, sqrt
import math as m
from src import Circuit, CoordinateSystem, VoltageSource, Wire, World


if __name__ == "__main__":
    # INITIALISATION DES PARAMETRES DE BASE
    WORLD_SHAPE = (101, 101)
    BATTERY_VOLTAGE = 1.0
    HIGH_WIRE_RESISTANCE = 1.0
    LOW_WIRE_RESISTANCE = 0.01

    # INITIALISATION DES VARIABLES
    cartesian_variables = Symbol("x"), Symbol("y")
    x, y = cartesian_variables

    # INITIALISATION DES EQUATIONS PARAMETRIQUES
    x_expression_droite_h = x
    y_expression_droite_h = y
    droite_h = (x_expression_droite_h, y_expression_droite_h)
    
    # CREATION DES PARTIES DU CIRCUIT
    wires = [
        Wire((50, 25), (40, 38), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE),           # FIL 1
        VoltageSource((40, 38), (35, 42), droite_h, cartesian_variables, BATTERY_VOLTAGE),      # SOURCE
        Wire((35, 42), (25, 50), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE),           # FIL 2  
        Wire((25, 50), (38, 75), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE),           # FIL 3
        Wire((38, 75), (50, 70), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE),           # FIL 4
        Wire((50, 70), (62, 60), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE),           # FIL 5
        Wire((62, 60), (70, 50), droite_h, cartesian_variables, HIGH_WIRE_RESISTANCE),          # RESISTANCE 1
        Wire((70, 50), (75, 43), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE),           # FIL 6
        Wire((75, 43), (75, 38), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE),           # FIL 7
        Wire((75, 38), (50, 25), droite_h, cartesian_variables, LOW_WIRE_RESISTANCE)            # FIL 8
    ]

    # GROUND
    ground_position = (40, 38)

    # CREATION DU CIRCUIT ET DE LA MAP
    circuit = Circuit(wires, ground_position)
    world = World(circuit=circuit, coordinate_system=CoordinateSystem.CARTESIAN, shape=WORLD_SHAPE)
    
    # AFFICHAGE DU CIRCUIT
    world.show_circuit(
        {0: (50, 25), 1: (40, 38), 2: (35, 42), 3: (38, 75), 4: (50, 70), 5: (62, 60), 6: (70, 50), 7: (75, 43), 8: (75, 38), 9: (50, 25)}
    )
    
    # CALCULS DES CHAMPS
    world.compute()
    
    # AFFICHAGE DES CHAMPS
    world.show_all()