#include "mymath.h"

#define MIN_VAL (1E-7)

vec3f O31={0,0,0};
vec3f O61={0};
vec3f I31={1,1,1};
vec12f O12_1={0};
vec15f O15_1={0};
mat3f O33={{0,0,0},{0,0,0},{0,0,0}};
mat3f I33={{1,0,0},{0,1,0},{0,0,1}};
mat6f I66={{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
mat6f O66={0};
mat12_6f O12_6={0};
mat12f O12_12={0};
mat15f O15_15={0};
mat15_6f O15_6={0};
mat6_15f O6_15={0};

bool is_zero(float val){
	return (fabs(val)<MIN_VAL)?true:false;
}

bool is_equal(float val_1, float val_2){
	return (fabs(val_1-val_2)<MIN_VAL)?true:false;
}

float int2float(float *p_f,u8 *p_u8){
	int2f_t i2f;
	for(u8 i=0;i<4;i++){
		i2f.u8vals[i] = p_u8[3-i];
	}
	*p_f = i2f.fval;
	return i2f.fval;
}

float norm(float x, float y){
	return sqrt(x*x+y*y);
}


float length_of_vec3f(vec3f vec){
	return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}


float constrain_float(float val, float min, float max){
	if(val<min) return min;
	if(val>max) return max;
	return val;
}

//��ʾ����
void print_mat(arm_matrix_instance_f32 mat){
	float *p=mat.pData,*pEnd=mat.pData+mat.numCols;
	for(int i=0;i<mat.numRows;++i){
		for(;p<pEnd;p++)	printf("%.5f ",*p);
		printf("\r\n");
		pEnd += mat.numCols;
	}
}
void print_mat_name(arm_matrix_instance_f32 mat, const char* mat_name){
	printf("====%s====\r\n",mat_name);
	float *p=mat.pData,*pEnd=mat.pData+mat.numCols;
	for(int i=0;i<mat.numRows;++i){
		for(;p<pEnd;p++)	printf("%.5f ",*p);
		printf("\r\n");
		pEnd += mat.numCols;
	}
}

//һά����->ʸ��
void set2vecf(float* data,u16 dim,float *vec,arm_matrix_instance_f32 *mat){
	memcpy(vec,data,dim*4);
	arm_mat_init_f32(mat ,dim,1,vec);
}

//����ƽ��->ʸ��
void set2vecf_sq(float *data,u16 dim,float *vec,arm_matrix_instance_f32 *mat){
	for(int i=0;i<dim;++i){
		*(vec+i) = (*(data+i))*(*(data+i));
	}
	arm_mat_init_f32(mat ,dim,1,vec);
}
//���ݱ���->ʸ��
void set2vecf_scale(float *data, float scale, u16 dim, float *vec,arm_matrix_instance_f32 *mat){
	for(int i=0;i<dim;++i){
		*(vec+i) = (*(data+i))*scale;
	}
	arm_mat_init_f32(mat ,dim,1,vec);
}
/*ע��
	�ڴ��ݶ�ά����ʱ�����ܲ�������float**�ķ�ʽ��
	���磬����
	mat3f m; func(float **m);
	�۲��ά�����ַ������m�ĵ�ַ��m[0]�ĵ�ַһ������ʱ��*m��m�ĵ�һ������ã�
		��ʵ��ʾ����ָ���һ�е�����ָ�룬��m[0]���ٴ�*(*m)�ű�ʾ�����׸�Ԫ�ء�
	����ά�����ڴ��ݵ����������˻�Ϊָ�룬��ʱfunc�е�*m���Ѿ���ʾm[0][0]�ˣ�
		�ٴ�*(*m)û���κ����壬�������ܸ��������ڴ����������
	->�²��Ǳ�������ʶ���Ƕ�ά����ʱ�����ֽţ�
*/
//��ά����->����
void set2matf(float* data,u16 rows,u16 cols,float *vec,arm_matrix_instance_f32 *mat){
	memcpy(vec,data,rows*cols*4);
	arm_mat_init_f32(mat,rows,cols,vec);
}

//����->�Խ�������(����)
void set2diag(float *data,u16 rows,float *diag,arm_matrix_instance_f32 *mat){
	for(int i=0;i<rows;++i){
		for(int j=0;j<rows;++j){
			if(i==j) *(diag+i*rows+j) = *(data+i);
			else *(diag+i*rows+j) = 0;
		}
	}
	arm_mat_init_f32(mat ,rows,rows,diag);
}
//����ƽ��->�Խ�������(����)
void set2diag_sq(float *data,u16 rows,float *diag,arm_matrix_instance_f32 *mat){
	for(int i=0;i<rows;++i){
		for(int j=0;j<rows;++j){
			if(i==j) *(diag+i*rows+j) = (*(data+i))*(*(data+i)); 
			else *(diag+i*rows+j) = 0;
		}
	}
	arm_mat_init_f32(mat ,rows,rows,diag);
}
//��3*3�ľ���src��ֵ����һ������des��(i,j)λ����
void setMat3(mat3f src,arm_matrix_instance_f32 *mat,int i,int j){
	for(int m=0;m<3;++m){
		for(int n=0;n<3;++n){
			*((*mat).pData+(i+m)*mat->numCols+(j+n)) = src[m][n];
		}
	}
}
//��һ��row*cols�ľ���ֵ����һ������des��(i,j)λ����->ǰ����des�Ѿ���ʼ��
void setMat(float *src, int row, int cols, arm_matrix_instance_f32 *des,int i,int j){
//	assert(des->pData==0);
	for(int m=0;m<row;++m){
		for(int n=0;n<cols;++n){
			*(des->pData+(i+m)*des->numCols+j+n) = *(src+m*cols+n);
		}
	}
}
//��һ������ֵ����һ������des��(i,j)λ����
void setMat2Mat(arm_matrix_instance_f32 *src,arm_matrix_instance_f32 *des,int i,int j){
//	assert(src->numCols==src.numRows);
	for(int m=0;m<src->numRows;++m){
		for(int n=0;n<src->numCols;++n){
			*(des->pData+(i+m)*des->numCols+j+n) = *(src->pData+m*src->numCols+n);
		}
	}
}


//����3�׷��Գƾ���
void askew(vec3f data,arm_matrix_instance_f32 *mat){
	mat3f temp = {
		{0,			-data[2],	data[1]},
		{data[2],	0,			-data[0]},
		{-data[1],	data[0],	0}	};
	memcpy((*mat).pData,*temp,3*3*4);
}
//����ȡ��
void op_mat(float *src,float *des,	int rows, int cols){
	for(int i=0;i<rows;++i)
		for(int j=0;j<cols;++j){
			*(des+i*cols+j) = -*(src+i*cols+j);
		}
}
//����ӵ�λ��
void add_eye2mat(arm_matrix_instance_f32 *mat){
	int dims = mat->numCols;
	for(int i=0;i<dims;++i)
		for(int j=0;j<dims;++j){
			if(i==j) *(mat->pData+i*dims+j) += 1;
		}
}
//�ж�ʸ���Ƿ�Ϊ0
bool vecf_isZero(float *vec, int dims){
	for(int i=0;i<dims;++i){
		float temp = *(vec+i);
		if(temp>EPS || temp<-EPS) return false;
	}
	return true;
}
//����Գƻ�
void symmetry(float *mat, int dims){
	for(int i=0; i<dims; i++){
		float *prow=mat+i*dims+i+1;	//mat[i][i+1]
		float *prowEnd=mat+i*dims+dims;//mat[i][dims]
		float *pclm=mat+i*dims+i+dims;//mat[i+1][i]
		for(; prow<prowEnd; prow++,pclm+=dims)  
//			*prow=*pclm=(*prow+*pclm)*0.5f;
			*prow=*pclm=0;
	}
}

void mymath_test(){
//set2vecf
	printf("====set2vecf====\r\n");
	vec3f v1_data,data2={1,2,3};
	arm_matrix_instance_f32 v1;
	set2vecf(data2,3,v1_data,&v1);
	print_mat(v1);
//set2vecf_sq
	printf("====set2vecf_sq====\r\n");
	print_mat(v1);
	set2vecf_sq(data2,3,v1_data,&v1);
	printf("----->\r\n");
	print_mat(v1);
//set2vecf_scale
	printf("====set2vecf_scale====\r\n");
	print_mat(v1);
	set2vecf_scale(data2,0.5,3,v1_data,&v1);
	printf("----->\r\n");
	print_mat(v1);
//set2matf
	printf("====set2matf====\r\n");
	mat3f m1_data,data1={{1,2,3},{4,5,6},{7,8,9}};
	arm_matrix_instance_f32 m1;
	set2matf(*data1,3,3,*m1_data,&m1);
	print_mat(m1);
//set2diag
	printf("====set2diag====\r\n");
	mat3f m2_data;
	arm_matrix_instance_f32 m2;
	set2diag(data2,3,*m2_data,&m2);
	print_mat(m2);
//set2diag_sq
	set2diag_sq(data2,3,*m2_data,&m2);
	printf("====set2diag_sq====\r\n");
	print_mat(m2);
//setMat3
	printf("====setMat3====\r\n");
	mat6f m3_data={0};
	arm_matrix_instance_f32 m3;
	arm_mat_init_f32(&m3,6,6,*m3_data);
	setMat3(m2_data,&m3,0,0);
	setMat3(I33,&m3,3,3);
	print_mat(m3);
//askew
	printf("====askew====\r\n");
	askew(data2,&m2);
	print_mat(m2);
//op_mat
	printf("====op_mat====\r\n");
	print_mat(m1);
	op_mat(m1.pData, m1.pData, m1.numRows, m1.numCols);
	printf("----->\r\n");
	print_mat(m1);
//add_eye2mat
	printf("====add_eye2mat====\r\n");
	print_mat(m1);
	printf("----->\r\n");
	add_eye2mat(&m1);
	print_mat(m1);
//vecf_isZero
	printf("====vecf_isZero====\r\n");
	printf("res= %d %d \r\n",vecf_isZero(data2, 3), vecf_isZero(O31, 3));
//symmetry
	printf("====symmetry====\r\n");
	print_mat(m1);
	symmetry(m1.pData, m1.numCols);	
	printf("----->\r\n");
	print_mat(m1);

}


