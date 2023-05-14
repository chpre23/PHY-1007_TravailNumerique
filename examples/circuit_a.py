import env_examples  # Modifies path, DO NOT REMOVE
from sympy import Symbol
from src.circuit import Circuit
from src.electrical_components import VoltageSource, Wire
from src.coordinate_and_position import CoordinateSystem
from src.world import World


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
    x_expression_vertical = 0 * x 
    y_expression_vertical = y
    vertical_eqs = (x_expression_vertical, y_expression_vertical)

    x_expression_horizontal = x
    y_expression_horizontal = 0 * y
    horizontal_eqs = (x_expression_horizontal, y_expression_horizontal)

    # CREATION DES PARTIES DU CIRCUIT
    wires = [
        Wire((30, 25), (30, 125), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),         # FIL GAUCHE
        Wire((30, 125), (90, 125), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # FIL HAUT LR
        Wire((90, 125), (105, 125), horizontal_eqs, cartesian_variables, HIGH_WIRE_RESISTANCE),    # FIL HAUT HR
        Wire((105, 125), (120, 125), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),    # FIL HAUT LR
        Wire((120, 125), (120, 60), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),       # FIL DROITE LR
        Wire((120, 60), (120, 45), vertical_eqs, cartesian_variables, HIGH_WIRE_RESISTANCE),       # FIL DROITE HR
        Wire((120, 45), (120, 25), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),        # FIL DROITE LR
        Wire((120, 25), (60, 25), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),       # FIL BAS LR
        Wire((40, 25), (30, 25), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),        # FIL BAS LR
        VoltageSource((60, 25), (40, 25), horizontal_eqs, cartesian_variables, BATTERY_VOLTAGE)    # SOURCE
    ]
    # GROUND
    ground_position = (60, 25)

    # CREATION DU CIRCUIT ET DE LA MAP
    circuit = Circuit(wires, ground_position)
    world = World(circuit=circuit, coordinate_system=CoordinateSystem.CARTESIAN, shape=WORLD_SHAPE)
    
    # AFFICHAGE DU CIRCUIT
    world.show_circuit(
        {0: (30, 25), 1: (30, 125), 2: (90, 125), 3: (105, 125), 4: (120, 125), 5: (120, 60), 6: (120, 45), 7: (120, 25), 8: (60, 25), 9: (40, 25)}
    )
    
    # CALCULS DES CHAMPS
    world.compute()
    
    # AFFICHAGE DES CHAMPS
    world.show_all()