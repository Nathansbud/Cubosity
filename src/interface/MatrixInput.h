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

    void insertInputRow(int row) {
        this->insertRow(row);
        for(int i = 0; i < this->columnCount(); i++) {
            QLineEdit* cell = new QLineEdit;
            cell->setText((i == this->columnCount() - 1) ? "1" : "0");
            cell->setValidator(new QDoubleValidator(cell));
            connect(cell, &QLineEdit::textChanged, this, [=](auto) {
               emit cellModified(row, i);
            });

            this->setCellWidget(row, i, cell);
        }
    }

    void populateValues(MatrixXd& mat) {
        for(int r = 0; r < mat.rows(); r++) {
            for(int c = 0; c < mat.cols(); c++) {
                static_cast<QLineEdit*>(this->cellWidget(r, c))->setText(QString::number(mat(r, c)));
            }
        }
    }
signals:
    void cellModified(int r, int c);
};
