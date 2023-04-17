#pragma once
#include "UIUtil.h"

#include <QBoxLayout>
#include <QColor>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QString>
#include <QColorDialog>

#include <iostream>

class OrientationGroup : public QWidget
{
    Q_OBJECT
public:
    QHBoxLayout* contents;
    QPushButton* colorButton;
    QDoubleSpinBox* lambda;

    QColor originalColor;

    OrientationGroup(QWidget *parent = nullptr) {
        this->originalColor = QColor::fromRgb(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
        this->colorButton = new QPushButton("");
        this->colorButton->setStyleSheet(
            QString("background-color: %1; border: none;").arg(originalColor.name())
        );

        this->lambda = UIUtil::makeDoubleSpinBox(0.1, 10, 0.1, 1.0, 1);

        this->contents = new QHBoxLayout();
        this->contents->addWidget(colorButton);
        this->contents->addWidget(lambda);
        this->setLayout(contents);

        connect(this->colorButton, &QPushButton::clicked, this, [&]() {
            QColor ret = QColorDialog::getColor();
            this->colorButton->setStyleSheet(
                QString("background-color: %1; border: none;").arg(ret.name())
            );
        });

        connect(this->lambda, &QDoubleSpinBox::valueChanged, this, [&](double v) {
            std::cout << "i cbf to deal with wiring all this up rn" << std::endl;
        });
    }
};
