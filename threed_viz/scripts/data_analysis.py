def pearson_correlation(x, y):
    if len(x) != len(y):
        raise ValueError("The size of x and y must be the same.")
    
    sum_x = sum(x)
    sum_y = sum(y)
    sum_xy = sum(x[i] * y[i] for i in range(len(x)))
    sum_x2 = sum(x[i] ** 2 for i in range(len(x)))
    sum_y2 = sum(y[i] ** 2 for i in range(len(y)))
    
    mean_x = sum_x / len(x)
    mean_y = sum_y / len(y)
    
    numerator = sum_xy - (mean_x * sum_y)
    denominator = ((sum_x2 - (mean_x ** 2 * len(x))) * (sum_y2 - (mean_y ** 2 * len(y)))) ** 0.5
    
    if denominator == 0:
        raise ValueError("Denominator is zero, cannot divide by zero.")
    
    return numerator / denominator

# Example usage:
x = [10, 20, 30, 40, 50]
y = [50, 49, 48, 47, 46]

try:
    correlation = pearson_correlation(x, y)
    print("Pearson Correlation Coefficient:", correlation)
except Exception as e:
    print("Error:", e)