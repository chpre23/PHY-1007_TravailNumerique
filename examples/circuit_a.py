import env_examples  # Modifies path, DO NOT REMOVE
from sympy import Symbol
from src import Circuit, CoordinateSystem, VoltageSource, Wire, World


if __name__ == "__main__":
    WORLD_SHAPE = (150, 150)
    BATTERY_VOLTAGE = 1.0
    HIGH_WIRE_RESISTANCE = 1.0
    LOW_WIRE_RESISTANCE = 0.01

    cartesian_variables = Symbol("x"), Symbol("y")
    x, y = cartesian_variables

    x_expression_vertical = 0 * x 
    y_expression_vertical = y
    vertical_eqs = (x_expression_vertical, y_expression_vertical)

    x_expression_horizontal = x
    y_expression_horizontal = 0 * y
    horizontal_eqs = (x_expression_horizontal, y_expression_horizontal)

    wires = [
        Wire((30, 25), (30, 125), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # fil de gauche

        Wire((30, 125), (90, 125), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # fil du haut non résistif 1
        Wire((90, 125), (105, 125), horizontal_eqs, cartesian_variables, HIGH_WIRE_RESISTANCE),      # fil du haut résistif
        Wire((105, 125), (120, 125), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # fil du haut non résistif 2

        Wire((120, 125), (120, 60), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # fil de droite non résistif 1
        Wire((120, 60), (120, 45), vertical_eqs, cartesian_variables, HIGH_WIRE_RESISTANCE),      # fil de droite résistif
        Wire((120, 45), (120, 25), vertical_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # fil de droite non résistif 2

        Wire((120, 25), (60, 25), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # fil du bas non résistif 1
        Wire((40, 25), (30, 25), horizontal_eqs, cartesian_variables, LOW_WIRE_RESISTANCE),      # fil du bas non résistif 2
        VoltageSource((60, 25), (40, 25), horizontal_eqs, cartesian_variables, BATTERY_VOLTAGE)      # source de tension
    ]
    ground_position = (60, 25)

    circuit = Circuit(wires, ground_position)
    world = World(circuit=circuit, coordinate_system=CoordinateSystem.CARTESIAN, shape=WORLD_SHAPE)
    world.show_circuit(
        {0: (30, 25), 1: (30, 125), 2: (90, 125), 3: (105, 125), 4: (120, 125), 5: (120, 60), 6: (120, 45), 7: (120, 25), 8: (60, 25), 9: (40, 25)}
    )
    world.compute()
    world.show_all()