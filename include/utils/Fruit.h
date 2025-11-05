#pragma once

#include <string>

class Fruit {
 private:
  std::string name;
  double nutritionalValue;
  bool fresh;

 public:
  Fruit() : name("Unknown"), nutritionalValue(0.0), fresh(true) {}
  Fruit(const std::string& name, double nutrition, bool isFresh)
      : name(name), nutritionalValue(nutrition), fresh(isFresh) {}

  Fruit(const Fruit& other) = default;

  Fruit& operator=(const Fruit& other) = default;

  ~Fruit() = default;

  const std::string& getName() const { return name; }
  double getNutritionalValue() const { return nutritionalValue; }
  bool isFresh() const { return fresh; }

  void setName(const std::string& newName) { name = newName; }
  void setNutritionalValue(double value) { nutritionalValue = value; }
  void setFresh(bool isFresh) { fresh = isFresh; }

  // Utility methods
  void spoil() { fresh = false; }
  void restore() { fresh = true; }

  // Comparison operators
  bool operator==(const Fruit& other) const {
    return name == other.name && nutritionalValue == other.nutritionalValue &&
           fresh == other.fresh;
  }

  bool operator!=(const Fruit& other) const { return !(*this == other); }
};
