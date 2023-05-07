#pragma once

#include <QTableWidget>
#include <QLineEdit>
#include <QHeaderView>
#include <Eigen/Dense>


using namespace Eigen;

class MatrixInput : public QTableWidget
{
    Q_OBJECT
public:
    MatrixInput(MatrixXd& matrix, int rows, int cols, QWidget *parent = nullptr) {
        this->setRowCount(rows);
        this->setColumnCount(cols);
        this->verticalHeader()->setVisible(false);
        this->horizontalHeader()->setVisible(false);
        this->showMinimized();

        for(int r = 0; r < rows; r++) {
            for(int c = 0; c < cols; c++) {
                QLineEdit* cell = new QLineEdit;
                cell->setText(QString::number(matrix(r, c)));
                cell->setValidator(new QDoubleValidator(cell));
                connect(cell, &QLineEdit::textChanged, this, [=](auto) {
                   emit cellModified(r, c);
                });

                this->setCellWidget(r, c, cell);
            }
        }
    }
signals:
    void cellModified(int r, int c);
};
