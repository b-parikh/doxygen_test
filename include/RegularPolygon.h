#ifndef POLYGON_H
#define POLYGON_H

/**
 * @file RegularPolygon.h
 * @brief Defines a class RegularPolygon and it's derivatives
 */

#include <iostream>
#include <string>

/** 
 * @brief An interface to represent regular polygons.
 *
 * This is a larger description of the class. 
 * If the user inherits from this class, they must implement the following methods:
 *   - calculateArea
 */
class RegularPolygon {
public:

    /** @brief An enum.
     *
     *  For additional documentation, make sure to add an empty line as above. 
     */ 
    enum COLOR {
        RED,   /**< enum value 1 */
        BLUE,  /**< enum value 2 */
        GREEN, /**< enum value 3 */
        BLACK  /**< enum value 4 */
    };

    /** @brief Calculates area of the polygon. Purely virtual and must be implemented by children class.
     *  @return Area of polygon
     */
    virtual float calculateArea() = 0;

    /**
     * @brief getter for shape color
     * @return shape's color
     */
    COLOR getColor() { return color_; };

    /**
     * @brief getter for shape type
     * @return type of shape
     */
    std::string getType() { return type_; };

    /**
     * @brief getter for shape's side length
     * @return side length of shape
     */
    float getSideLength() { return sideLength_; }

protected:
    std::string type_; /**< The type of polygon. Eg. triangle, square, etc. */
    COLOR color_; /**< The color of polygon. Defined using the COLOR enum. */
    float sideLength_; /**< All regular polygons have equidistant side lengths. */
};

/** 
 * @brief A Triangle class derived from the RegularPolygon interface.
 */
class Triangle : public RegularPolygon {
public:
    /**
     * @brief Constructor
     *
     * @param[in] sideLength - (optional) triangle's side length (default value is 1)
     * @param[in] color - (optional) triangle's color (default value is COLOR::BLACK)
     *
     * @return Triangle instance
     */
    Triangle(int sideLength=1, COLOR color=BLACK) {
        this->type_ = "triangle";
        this->sideLength_ = sideLength;
        this->color_ = color;
    }


    /**
     * @brief Calculates the area of an equilaterial triangle.
     *
     * This function uses the formula sqrt(3)/4 * sideLength to calculate the area.
     *
     * @return area of triangle
     */
    float calculateArea() override {
        return sideLength_ * 0.433013;  // sideLength * sqrt(3)/4
    }

    /**
     * @return triangle side length
     */
    int getSidelength() { return sideLength_; }

private:

    /**
     * @brief Private default constructor ensures that user must use provided constructor.
     */
    Triangle() {}

};

/** 
 * @brief A Square class derived from the RegularPolygon interface.
 */
class Square : public RegularPolygon {
public:
    /**
     * @brief Constructor
     *
     * @param[in] sideLength - (optional) square's side length (default value is 1)
     * @param[in] color - (optional) square's color (default value is COLOR::BLACK)
     *
     * @return Square instance
     */
    Square(int sideLength=1, COLOR color=BLACK) {
        this->type_ = "square";
        this->sideLength_ = sideLength;
        this->color_ = color;
    }


    /**
     * @brief Calculates the area of a square.
     *
     * @return area of square
     */
    float calculateArea() override {
        return sideLength_ * sideLength_;
    }

    /**
     * @return square side length 
     */
    int getSidelength() { return sideLength_; }

private:

    /**
     * @brief Private default constructor ensures that user must use provided constructor.
     */
    Square() {}

};

#endif
