//
// Created by robotics-verse on 04.05.22.
//
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

class ContourProperties {

public:
    ContourProperties(const cv::Point_<float>& center, float radius, bool dummy_value = false) {
        _center = center;
        _radius = radius;
        _dummy_value = dummy_value;
        _area = M_PI * radius * radius;
    }


    const cv::Point_<float> &getCenter() const {
        return _center;
    }

    void setCenter(const cv::Point_<float> &center) {
        _center = center;
    }

    float getRadius() const {
        return _radius;
    }

    void setRadius(float radius) {
        _radius = radius;
    }

    bool isDummyValue() const {
        return _dummy_value;
    }

    void setDummyValue(bool dummyValue) {
        _dummy_value = dummyValue;
    }

    float getArea() const {
        return _area;
    }

    void setArea(float area) {
        _area = area;
    }

    bool operator==(const ContourProperties &rhs) const {
        return _center == rhs._center &&
               _radius == rhs._radius &&
               _dummy_value == rhs._dummy_value &&
               _area == rhs._area;
    }

    bool operator!=(const ContourProperties &rhs) const {
        return !(rhs == *this);
    }


    cv::Point_<float> _center;
    float _radius;
    bool _dummy_value;
    float _area;
};
