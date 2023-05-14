
import env_examples  # Modifies path, DO NOT REMOVE
from sympy import Symbol, sqrt
import numpy as np
from numpy import cos, sin, pi
from src import Circuit, CoordinateSystem, VoltageSource, Wire, World


if __name__ == "__main__":
    # INITIALISATION DES PARAMETRES DE BASE
    WORLD_SHAPE = (85, 85)
    BATTERY_VOLTAGE = 1.0
    HIGH_WIRE_RESISTANCE = 1.0
    LOW_WIRE_RESISTANCE = 0.01
    
    # INITIALISATION DES VARIABLES
    polar_variables = Symbol("x"), Symbol("y")
    r, theta = polar_variables
    
    # INITIALISATION DES EQUATIONS PARAMETRIQUES
    r_expression_droite_h = r
    theta_expression_droite_h = theta
    expression_shape_weird = (r_expression_droite_h, theta_expression_droite_h)
    
    # CREATION DES PARTIES DU CIRCUIT
    wires = [
        Wire((30, 70), (50, 70), expression_shape_weird, polar_variables, LOW_WIRE_RESISTANCE),             # FIL HAUT DROITE
        Wire((50, 70), (50, 50), expression_shape_weird, polar_variables, LOW_WIRE_RESISTANCE),             # FIL CERCLE EXTERNE
        Wire((50, 50), (50, 40), expression_shape_weird, polar_variables, HIGH_WIRE_RESISTANCE),            # RESISTANCE CERCLE EXTERNE
        Wire((50, 40), (50, 15), expression_shape_weird, polar_variables, LOW_WIRE_RESISTANCE),             # FIL CERCLE EXTERNE
        Wire((50, 15), (30, 15), expression_shape_weird, polar_variables, LOW_WIRE_RESISTANCE),             # FIL BAS DROITE
        Wire((30, 15), (30, 40), expression_shape_weird, polar_variables, LOW_WIRE_RESISTANCE),             # FIL CERCLE INTERNE
        VoltageSource((30, 40), (30, 50), expression_shape_weird, polar_variables, BATTERY_VOLTAGE),        # SOURCE
        Wire((30, 50), (30, 70), expression_shape_weird, polar_variables, LOW_WIRE_RESISTANCE)              # FIL CERCLE INTERNE
    ]

    # GROUND
    ground_position = (50, 15)
    
    # CREATION DU CIRCUIT ET DE LA MAP
    circuit = Circuit(wires, ground_position)
    world = World(circuit=circuit, coordinate_system=CoordinateSystem.CARTESIAN, shape=WORLD_SHAPE)
    
    # AFFICHAGE DU CIRCUIT
    world.show_circuit(
        {0: (50, 70), 1: (70, 70), 2: (70, 50), 3: (70, 40), 4: (70, 15), 5: (50, 15), 6: (50, 40), 7: (50, 50)}
    )
    
    # CALCULS DES CHAMPS
    world.compute()
    
    # AFFICHAGE DES CHAMPS
    world.show_all()  