program assignments
  implicit none
  ! explicit integer declaration
  integer :: I, ii, jj
  integer :: K, kk, ll
  integer :: M, mm

  ! task-to-rank assignment matrices
  logical, allocatable :: xi_l(:,:)
  logical, allocatable :: xi_t(:,:)
  integer, allocatable :: xi_i(:,:)

  ! comm-to-task assignment tensors
  logical, allocatable :: sigma_l(:,:,:)
  integer, allocatable :: sigma_i(:,:,:)

  ! comm-to-rank assignment tensors
  logical, allocatable :: tau_l(:,:,:)
  integer, allocatable :: tau_i(:,:,:)

  ! intermediate matrix
  logical, allocatable :: xi_sigma_l(:,:)

  ! tensor bounds
  integer, allocatable :: tau_lb_i(:,:,:)
  integer, allocatable :: tau_ub1_i(:,:,:)
  integer, allocatable :: tau_ub2_i(:,:,:)

  ! sums in paper formulas
  integer :: sums(4)

  ! integer to string conversion
  character(32) :: int_to_str
  print *
  print *, "### Program assignments"
  print *

  ! allocate assignments
  I = 2
  K = 3
  M = 4
  allocate(xi_l(I, K))
  allocate(xi_t(I, K))
  allocate(xi_i(I, K))
  allocate(sigma_l(K, K, M))
  allocate(sigma_i(K, K, M))
  allocate(xi_sigma_l(I, K))
  allocate(tau_l(I, I, M))
  allocate(tau_i(I, I, M))
  allocate(tau_lb_i(I, I, M))
  allocate(tau_ub1_i(I, I, M))
  allocate(tau_ub2_i(I, I, M))

  ! populate and print task-rank matrices
  xi_l(1,:) =  [.TRUE. , .TRUE. , .FALSE.]
  xi_l(2,:) =  [.FALSE., .FALSE., .TRUE. ]
  xi_t = transpose(xi_l)
  xi_i = merge(1, 0, xi_l)
  call print_logical_matrix("xi", I, K, xi_l)
  call print_integer_matrix("xi", I, K, xi_i)
  print *

  ! populate and print communication-task tensors
  sigma_l = .FALSE.
  sigma_l(3,1,1) = .TRUE.
  sigma_l(3,2,2) = .TRUE.
  sigma_l(2,3,3) = .TRUE.
  sigma_l(2,1,4) = .TRUE.
  sigma_i = merge(1, 0, sigma_l)
  do mm = 1, M
     call print_logical_matrix("sigma::"//trim(int_to_str(mm)), K, K, sigma_l(:,:,mm))
  end do
  do mm = 1, M
     call print_integer_matrix("sigma::"//trim(int_to_str(mm)), K, K, sigma_i(:,:,mm))
  end do
  print *
  
  ! compute and print communication-rank tensors
  tau_i = merge(1, 0, tau_l)
  do mm = 1, M
     xi_sigma_l = matmul(xi_l, sigma_l(:,:,mm))
     tau_l(:,:,mm) = matmul(xi_sigma_l, xi_t)
     call print_logical_matrix("tau::"//trim(int_to_str(mm)), I, I, tau_l(:,:,mm))
  end do
  do mm = 1, M
     tau_i = merge(1, 0, tau_l)
     call print_integer_matrix("tau::"//trim(int_to_str(mm)), I, I, tau_i(:,:,mm))
  end do
  print *

  ! generate table
  print *, "------------------------------------"
  print *, "  m   j   i   l   k sklm xik xjl xxs"
  print *, "------------------------------------"
  ! iterate over tensor slices
  do mm = 1, M
     ! iterate over from rank indices
     do jj = 1, I
        ! iterate over to rank indices
        do ii = 1, I
           ! initialize sums
           sums = 0

           ! iterate over from task indices
           do ll = 1, K
              ! iterate over to task indices
              do kk = 1, K
                 ! update sums
                 sums(1) = sums(1) + xi_i(ii,kk) * xi_i(jj,ll) * sigma_i(kk,ll,mm) 
                 sums(2) = sums(2) + xi_i(ii,kk) * sigma_i(kk,ll,mm) 
                 sums(3) = sums(3) + xi_i(jj,ll) * sigma_i(kk,ll,mm) 
                 sums(4) = sums(4) + (xi_i(ii,kk) + xi_i(jj,ll)) * sigma_i(kk,ll,mm)
                 
                 ! print innermost loop results
                 print "(I4,I4,I4,I4,I4,I4,I4,I4,I4)", &
                      & mm, jj, ii, ll, kk, sigma_i(kk,ll,mm), xi_i(ii,kk), xi_i(jj,ll), &
                      & xi_i(ii,kk) * xi_i(jj,ll) * sigma_i(kk,ll,mm)
              end do ! kk
           end do ! ll

           ! store and print results aggregated at i,j level
           tau_ub1_i(ii,jj,mm) = sums(2)
           tau_ub2_i(ii,jj,mm) = sums(3)
           tau_lb_i(ii,jj,mm) = sums(4) - 1
           print *, "   ---------------------------------"
           print *, " tau_ijm Sxxs Sxiks Sxjls S(x+x)s-1"
           print "(I6,I6,I6,I6,I6)", tau_i(ii,jj,mm), sums(1), tau_lb_i(ii,jj,mm), tau_ub1_i(ii,jj,mm), tau_ub2_i(ii,jj,mm)
           print *, "   ---------------------------------"
        end do ! jj
     end do ! ii
     print *, "------------------------------------"
  end do ! mm
  
  ! print tensor bounds
  do mm = 1, M
     call print_integer_matrix("tau_lb::"//trim(int_to_str(mm)), I, I, tau_lb_i(:,:,mm))
  end do
  print *
  do mm = 1, M
     call print_integer_matrix("tau_ub1::"//trim(int_to_str(mm)), I, I, tau_ub1_i(:,:,mm))
  end do
  print *
  do mm = 1, M
     call print_integer_matrix("tau_ub2::"//trim(int_to_str(mm)), I, I, tau_ub2_i(:,:,mm))
  end do
  print *

  ! terminate program
  deallocate(tau_ub2_i)
  deallocate(tau_ub1_i)
  deallocate(tau_lb_i)
  deallocate(tau_i)
  deallocate(tau_l)
  deallocate(xi_sigma_l)
  deallocate(sigma_i)
  deallocate(sigma_l)
  deallocate(xi_i)
  deallocate(xi_t)
  deallocate(xi_l)
  print *, "End program assignments ###"
  print *

end program assignments
  
! print logical matrix to console
subroutine print_logical_matrix(str, n_rows, n_cols, mat)
  implicit none
  ! read-only input variables
  character(*), intent(in) :: str
  integer, intent(in) :: n_rows
  integer, intent(in) :: n_cols
  logical, intent(in) :: mat(n_rows, n_cols)

  ! iterate over matrix rows
  integer :: rr
  print *, "# Boolean ", str, " ="
  do rr = 1, n_rows
    print *, mat(rr, 1:n_cols)
  end do

end subroutine print_logical_matrix

! print integer matrix to console
subroutine print_integer_matrix(str, n_rows, n_cols, mat)
  implicit none
  ! read-only input variables
  character(*), intent(in) :: str
  integer, intent(in) :: n_rows
  integer, intent(in) :: n_cols
  integer, intent(in) :: mat(n_rows, n_cols)

  ! iterate over matrix rows
  integer :: rr
  print *, "# Integer ", str, " ="
  do rr = 1, n_rows
    print *, mat(rr, 1:n_cols)
  end do

end subroutine print_integer_matrix

! convert integer to string
function int_to_str(I) result(str)
  implicit none
  ! pure function with pointer output
  integer, intent(in) :: I
  character(*) :: str

  ! convert and adjust
  write (str, *) I
  str = adjustl(str)

end function int_to_str
