import matplotlib.pyplot as plt

with open("log.txt") as f:
    data = [float(x.strip()) for x in f]

low = int(min(data))
high = int(max(data))+ 2

plt.hist(data, bins=range(low, high), edgecolor="black")
plt.xlabel("Counted time")
plt.ylabel("Number of attempts")
plt.title("User accuracy in counting 15 seconds")
plt.show()
