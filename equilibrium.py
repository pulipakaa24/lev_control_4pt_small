import sympy as sp

g = 9.81
m = 6
z = sp.symbols('z')
term1 = (6.167266375e8*((0.4446789110e5)**2))/((0.2466906550e10*z + 0.6886569338e8)**2)
term2 = 0.3042813963e19*z*((0.4446789110e5)**2)/((0.2466906550e10*z + 0.6886569338e8)**3)
equation = sp.Eq(4*(term1 - term2), m*g)
solution = sp.solve(equation, z)
print("Equilibrium position (z):")
for sol in solution:
    if sol.is_real and sol > 0:
        print(sol.evalf())