/*
 * imgprocess.h
 *
 *  Created on: 2017年5月19日
 *      Author: cheny
 */

#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_

int MedianFilter(void *bitmap);

void img_find_middle(void);
void img_cross_search(void);
void img_circle_left_search(void);
void img_circle_right_search(void);
void img_smalls_search(void);
void img_barrier_search(void);

void Algorithm_Bak(void);

#endif /* IMGPROCESS_H_ */
