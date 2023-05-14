import env_examples  # Modifies path, DO NOT REMOVE
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
    x_expression_vertical = 0 * x
    y_expression_vertical = y
    vertical_eqs = (x_expression_vertical, y_expression_vertical)

    x_expression_horizontal = x
    y_expression_horizontal = 0 * y
    horizontal_eqs = (x_expression_horizontal, y_expression_horizontal)

    # CREATION DES PARTIES DU CIRCUIT
    wires = [
        Wire((120, 65), (120, 25), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),       # FIL DROIT

        Wire((120, 25), (90, 25), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # FIL BAS DROIT
        Wire((90, 25), (60, 25), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),       # FIL BAS CENTRE
        Wire((60, 25), (30, 25), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),       # FIL BAS GAUCHE
        Wire((30, 25), (30, 65), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),         # FIL BAS GAUCHE
        VoltageSource((30, 65), (30, 85), vertical_eqs, cartesian_variables, BATTERY_VOLTAGE),    # SOURCE GAUCHE
        Wire((30, 85), (30, 125), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),        # FIL GAUCHE
        Wire((30, 125), (60, 125), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),     # FIL HAUT GAUCHE
        Wire((60, 125), (90, 125), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),     # FIL HAUT CENTRE
        Wire((90, 125), (120, 125), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),    # FIL HAUT DROIT
        Wire((120, 125), (120, 85), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # FIL HAUT DROIT
        VoltageSource((120, 65), (120, 85), vertical_eqs, cartesian_variables, BATTERY_VOLTAGE),  # SOURCE DROITE
        Wire((60, 125), (60, 85), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),        # FIL GAUCHE HAUT
        Wire((60, 85), (60, 65), vertical_eqs, cartesian_variables, HIGH_WIRE_RESISTANCE),        # FIL CENTRE GAUCHE
        Wire((60, 65), (60, 25), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),         # FIL CENTRE GAICHE
        Wire((90, 125), (90, 85), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),        # FIL CENTRE DROIT HAUT
        Wire((90, 85), (90, 65), vertical_eqs, cartesian_variables, HIGH_WIRE_RESISTANCE),        # FIL CENTRE DROIT
        Wire((90, 65), (90, 25), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),         # FIL CENTRE DROIT BAS
    ]
    # GROUND
    ground_position = (120, 65)

    # CREATION DU CIRCUIT ET DE LA MAP
    circuit = Circuit(wires, ground_position)
    world = World(circuit=circuit, coordinate_system=CoordinateSystem.CARTESIAN, shape=WORLD_SHAPE)
    
    # AFFICHAGE DU CIRCUIT
    world.show_circuit(
        {0: (120, 65), 1:(120, 25), 2:(90, 25), 3: (60, 25), 4: (30, 25), 5: (30, 65), 6: (30, 85), 7: (30, 125),
        8: (60, 125), 9: (90, 125), 10: (120, 125), 11: (120, 85), 12: (60, 85), 13: (60, 65), 14: (90, 85), 15: (90, 65)}
    )
    
    # CALCULS DES CHAMPS
    world.compute()
    
    # AFFICHAGE DES CHAMPS
    world.show_all()