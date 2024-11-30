void end_effector_kinematics_velocity_jacobian_sparsity(unsigned long const** row,
                                                        unsigned long const** col,
                                                        unsigned long* nnz) {
   static unsigned long const rows[32] = {0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2};
   static unsigned long const cols[32] = {0,1,2,3,4,5,6,7,8,9,10,0,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10};
   *row = rows;
   *col = cols;
   *nnz = 32;
}
