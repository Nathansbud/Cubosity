#pragma once
#include "UIUtil.h"
#include "MatrixInput.h"

#include <QBoxLayout>
#include <QColor>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QString>
#include <QColorDialog>
#include <Eigen/Dense>

#include <iostream>

class OrientationGroup : public QWidget
{
    Q_OBJECT
public:
    QHBoxLayout* contents;
    QPushButton* colorButton;
    QDoubleSpinBox* lambda;

    MatrixInput* rotInput;

    Eigen::MatrixXf rotation;

    QColor color;
    int groupID;

    inline static int nextID = 0;

    OrientationGroup(QWidget *parent = nullptr) {
        this->color = QColor::fromRgb(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
        this->colorButton = new QPushButton("");
        this->colorButton->setStyleSheet(
            QString("background-color: %1; border: none;").arg(color.name())
        );

        this->lambda = UIUtil::makeDoubleSpinBox(0.1, 10, 0.1, 1.0, 1);
        this->rotation = Matrix3f::Identity();
        this->rotInput = new MatrixInput(rotation, 3, 3);

        this->contents = new QHBoxLayout();
        this->contents->addWidget(colorButton);
        this->contents->addWidget(lambda);
        this->contents->addWidget(rotInput);
        this->setLayout(contents);

        connect(this->colorButton, &QPushButton::clicked, this, [&]() {
            QColor ret = QColorDialog::getColor();
            this->colorButton->setStyleSheet(
                QString("background-color: %1; border: none;").arg(ret.name())
            );
            this->color = ret;
        });

        connect(this->lambda, &QDoubleSpinBox::valueChanged, this, [&](double v) {
            std::cout << "i cbf to deal with wiring all this up rn" << std::endl;
        });

        this->groupID = nextID++;
    }
};
