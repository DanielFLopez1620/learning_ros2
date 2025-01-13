"""
simple_excersize.py

Simple example where test are made for addition and substraction functions.
"""

# ---------------------- Functions definitions --------------------------------
def sum(n1, n2):
    """
    Addition of two numbers

    Params
    ---
    n1 : Integer or float number # 1
    n2 : Integer or float number # 2

    Returns
    ---
    n1 + n2
    """
    return n1 + n2

def sub(n1, n2):
    """
    Substraction of two numbers

    Params
    ---
    n1 : Integer or float number # 1
    n2 : Integer or float number # 2

    Returns
    ---
    n1 - n2
    """
    return n1 - n2

# ----------------------- Tests to implement ---------------------------------
def test_sum():
    """
    Assert a fixed sum
    """
    assert sum(16, 20) == 36

def test_sub():
    """
    Assert a fixed substraction
    """
    assert sub(16, 20) == -4

# --------------------- Main implementation ----------------------------------
def main():
    """
    Display the results of the basic operations of two numbers.
    """
    n1 = 4
    n2 = 3
    print(f"{n1} + {n2} = {sum(n1, n2)}")
    print(f"{n1} - {n2} = {sub(n1, n2)}")


if __name__ == '__main__':
    main()