#pragma once
#include <map>

enum {
	BLACK_1 = 0xA1428DD0,
	BLACK_2,
	BLUE_1,
	BLUE_2,
	BLUE_3,
	BLUE_4,
	BLUE_5,
	BLUE_6,
	PURPLE_2,
	PURPLE_3,
	RED_1,
	RED_2,
	RED_3,
	ORANGE_2,
	ORANGE_3,
};

typedef struct{
	unsigned int modell_name;
	float imu_acc_x_m1g;
	float imu_acc_x_p1g;
	float imu_acc_y_m1g;
	float imu_acc_y_p1g;
} minicube_t;

static uint64_t mcu_uid = *(uint64_t *)0x1FFF7590;

static std::map<unsigned long long, minicube_t> MINICUBE = {
	{0x533250060068003E, {.modell_name = BLACK_1, .imu_acc_x_m1g = -15965.00, .imu_acc_x_p1g = 16870.00, .imu_acc_y_m1g = -17208.00, .imu_acc_y_p1g = 15620.00}},
	{0x595650190030002B, {.modell_name = BLACK_2, .imu_acc_x_m1g = -16250.00, .imu_acc_x_p1g = 16630.00, .imu_acc_y_m1g = -17410.00, .imu_acc_y_p1g = 15380.00}},
	{0x59565019004F0034, {.modell_name = BLUE_1, .imu_acc_x_m1g = -16000.00, .imu_acc_x_p1g = 16830.00, .imu_acc_y_m1g = -17280.00, .imu_acc_y_p1g = 15480.00}},
	{0x59565018002D0065, {.modell_name = BLUE_2, .imu_acc_x_m1g = -16405.00, .imu_acc_x_p1g = 16480.00, .imu_acc_y_m1g = -17212.00, .imu_acc_y_p1g = 15600.00}},
	{0x595650190029002A, {.modell_name = BLUE_3, .imu_acc_x_m1g = -16200.00, .imu_acc_x_p1g = 16640.00, .imu_acc_y_m1g = -16540.00, .imu_acc_y_p1g = 16250.00}},
	{0x5956501900480036, {.modell_name = BLUE_4, .imu_acc_x_m1g = -16105.00, .imu_acc_x_p1g = 16775.00, .imu_acc_y_m1g = -17320.00, .imu_acc_y_p1g = 15515.00}},
	{0x59565019003D0032, {.modell_name = BLUE_5, .imu_acc_x_m1g = -16346.00, .imu_acc_x_p1g = 16589.00, .imu_acc_y_m1g = -17555.00, .imu_acc_y_p1g = 15301.00}},
	{0x5332500500230064, {.modell_name = BLUE_6, .imu_acc_x_m1g = -16180.00, .imu_acc_x_p1g = 16675.00, .imu_acc_y_m1g = -17155.00, .imu_acc_y_p1g = 15685.00}},
	{0x53325006004A0048, {.modell_name = PURPLE_2, .imu_acc_x_m1g = -16078.00, .imu_acc_x_p1g = 16770.00, .imu_acc_y_m1g = -16750.00, .imu_acc_y_p1g = 16100.00}},
	{0x59565018004F0064, {.modell_name = PURPLE_3, .imu_acc_x_m1g = -16115.00, .imu_acc_x_p1g = 16750.00, .imu_acc_y_m1g = -17517.00, .imu_acc_y_p1g = 15309.00}},
	{0x5956501800620065, {.modell_name = RED_1, .imu_acc_x_m1g = -16250.00, .imu_acc_x_p1g = 16650.00, .imu_acc_y_m1g = -17440.00, .imu_acc_y_p1g = 15440.00}},
	{0x53325006005A004A, {.modell_name = RED_2, .imu_acc_x_m1g = -15700.00, .imu_acc_x_p1g = 17140.00, .imu_acc_y_m1g = -17370.00, .imu_acc_y_p1g = 15480.00}},
	{0x5332500600300026, {.modell_name = RED_3, .imu_acc_x_m1g = -16200.00, .imu_acc_x_p1g = 16640.00, .imu_acc_y_m1g = -17290.00, .imu_acc_y_p1g = 15535.00}},
	{0x5332500600460037, {.modell_name = ORANGE_2, .imu_acc_x_m1g = -16200.00, .imu_acc_x_p1g = 16630.00, .imu_acc_y_m1g = -17474.00, .imu_acc_y_p1g = 15420.00}},
	{0x5956501900490036, {.modell_name = ORANGE_3, .imu_acc_x_m1g = -16040.00, .imu_acc_x_p1g = 16824.00, .imu_acc_y_m1g = -17380.00, .imu_acc_y_p1g = 15440.00}},
};
