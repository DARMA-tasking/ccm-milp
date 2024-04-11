!                           DARMA Toolkit v. 1.0.0
! 
! Copyright 2019 National Technology & Engineering Solutions of Sandia, LLC
! (NTESS). Under the terms of Contract DE-NA0003525 with NTESS, the U.S.
! Government retains certain rights in this software.
! 
! Redistribution and use in source and binary forms, with or without
! modification, are permitted provided that the following conditions are met:
! 
! * Redistributions of source code must retain the above copyright notice,
!   this list of conditions and the following disclaimer.
! 
! * Redistributions in binary form must reproduce the above copyright notice,
!   this list of conditions and the following disclaimer in the documentation
!   and/or other materials provided with the distribution.
! 
! * Neither the name of the copyright holder nor the names of its
!   contributors may be used to endorse or promote products derived from this
!   software without specific prior written permission.
! 
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
! POSSIBILITY OF SUCH DAMAGE.
! 
! Questions? Contact darma@sandia.gov
! 

program assignments
  implicit none
  ! explicit integer declaration
  integer :: I, ii, jj
  integer :: K, kk, ll
  integer :: M, mm
  integer :: N, nn

  ! block-to-task assignment matrices
  logical, allocatable :: u_l(:,:)
  integer, allocatable :: u_i(:,:)

  ! block-to-rank assignment matrices
  logical, allocatable :: phi_l(:,:)
  integer, allocatable :: phi_i(:,:)

  ! task-to-rank assignment matrices
  logical, allocatable :: chi_l(:,:)
  logical, allocatable :: chi_t(:,:)
  integer, allocatable :: chi_i(:,:)

  ! comm-to-task assignment tensors
  logical, allocatable :: w_l(:,:,:)
  integer, allocatable :: w_i(:,:,:)

  ! comm-to-rank assignment tensors
  logical, allocatable :: psi_l(:,:,:)
  integer, allocatable :: psi_i(:,:,:)

  ! intermediate matrix
  logical, allocatable :: chi_w_l(:,:)

  ! tensor bounds
  integer, allocatable :: psi_lb_i(:,:,:)
  integer, allocatable :: psi_ub1_i(:,:,:)
  integer, allocatable :: psi_ub2_i(:,:,:)

  ! sums in paper formulas
  integer :: sums(4)

  ! integer to string conversion
  character(32) :: int_to_str
  print *
  print *, "### Full Work Model Problem Example"
  print *

  ! allocate assignments
  I = 2
  K = 3
  M = 4
  N = 2
  allocate(u_l(K, N))
  allocate(u_i(K, N))
  allocate(phi_l(I, N))
  allocate(phi_i(I, N))
  allocate(chi_l(I, K))
  allocate(chi_t(I, K))
  allocate(chi_i(I, K))
  allocate(w_l(K, K, M))
  allocate(w_i(K, K, M))
  allocate(chi_w_l(I, K))
  allocate(psi_l(I, I, M))
  allocate(psi_i(I, I, M))
  allocate(psi_lb_i(I, I, M))
  allocate(psi_ub1_i(I, I, M))
  allocate(psi_ub2_i(I, I, M))

  ! populate and print parameters
  print *, "## Parameters:"
  print *, "I = ", int_to_str(I)
  print *, "K = ", int_to_str(K)
  print *, "M = ", int_to_str(M)
  print *, "N = ", int_to_str(N)
  print *
  u_l(1,:) = [.TRUE. , .FALSE.]
  u_l(2,:) = [.TRUE. , .FALSE.]
  u_l(3,:) = [.FALSE., .TRUE. ]
  call print_logical_matrix("u", K, N, u_l)
  call print_integer_matrix("u", K, N, u_i)
  print *
  w_l(3,1,1) = .TRUE.
  w_l(3,2,2) = .TRUE.
  w_l(2,3,3) = .TRUE.
  w_l(2,1,4) = .TRUE.
  w_i = merge(1, 0, w_l)
  do mm = 1, M
     call print_logical_matrix("w::"//trim(int_to_str(mm)), K, K, w_l(:,:,mm))
  end do
  do mm = 1, M
     call print_integer_matrix("w::"//trim(int_to_str(mm)), K, K, w_i(:,:,mm))
  end do
  print *

  ! populate and print variables
  print *, "## Variables:"
  chi_l(1,:) = [.TRUE. , .TRUE. , .FALSE.]
  chi_l(2,:) = [.FALSE., .FALSE., .TRUE. ]
  chi_t = transpose(chi_l)
  chi_i = merge(1, 0, chi_l)
  call print_logical_matrix("chi", I, K, chi_l)
  call print_integer_matrix("chi", I, K, chi_i)
  print *

  ! compute and print task-rank matrices
  phi_l = matmul(chi_l, u_l)
  phi_i = merge(1, 0, phi_l)
  call print_logical_matrix("phi", I, N, phi_l)
  call print_integer_matrix("phi", I, N, phi_i)
  print *
  
  ! compute and print communication-rank tensors
  psi_i = merge(1, 0, psi_l)
  do mm = 1, M
     chi_w_l = matmul(chi_l, w_l(:,:,mm))
     psi_l(:,:,mm) = matmul(chi_w_l, chi_t)
     call print_logical_matrix("psi::"//trim(int_to_str(mm)), I, I, psi_l(:,:,mm))
  end do
  do mm = 1, M
     psi_i = merge(1, 0, psi_l)
     call print_integer_matrix("psi::"//trim(int_to_str(mm)), I, I, psi_i(:,:,mm))
  end do
  print *

  ! generate integer communication tensor relations
  print *, "# Integer communication tensor relations:"
  print *, "------------------------------------------------------"
  print *, "m   j   i   l   k   w  chi chiT *   +  lb  psi ub1 ub2"
  print *, "------------------------------------------------------"
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
                 sums(1) = sums(1) + chi_i(ii,kk) * chi_i(jj,ll) * w_i(kk,ll,mm) 
                 sums(2) = sums(2) + chi_i(ii,kk) * w_i(kk,ll,mm) 
                 sums(3) = sums(3) + chi_i(jj,ll) * w_i(kk,ll,mm) 
                 sums(4) = sums(4) + (chi_i(ii,kk) + chi_i(jj,ll)) * w_i(kk,ll,mm)
                 
                 ! print innermost loop results
                 print "(I2,I4,I4,I4,I4,I4,I4,I4,I4)", &
                      & mm, jj, ii, ll, kk, w_i(kk,ll,mm), chi_i(ii,kk), chi_i(jj,ll), &
                      & chi_i(ii,kk) * chi_i(jj,ll) * w_i(kk,ll,mm)
              end do ! kk
           end do ! ll

           ! store and print results aggregated at i,j level
           psi_ub1_i(ii,jj,mm) = sums(2)
           psi_ub2_i(ii,jj,mm) = sums(3)
           psi_lb_i(ii,jj,mm) = sums(4) - 1
           print "(I38, I4, I4, I4, I4)", sums(1), &
                & psi_lb_i(ii,jj,mm), psi_i(ii,jj,mm), &
                & psi_ub1_i(ii,jj,mm), psi_ub2_i(ii,jj,mm)
        end do ! jj
     end do ! ii
     print *, "   --------------------------------------------------"
  end do ! mm
  
  ! print tensor bounds
  do mm = 1, M
     call print_integer_matrix("psi_lb::"//trim(int_to_str(mm)), I, I, psi_lb_i(:,:,mm))
  end do
  print *
  do mm = 1, M
     call print_integer_matrix("psi_ub1::"//trim(int_to_str(mm)), I, I, psi_ub1_i(:,:,mm))
  end do
  print *
  do mm = 1, M
     call print_integer_matrix("psi_ub2::"//trim(int_to_str(mm)), I, I, psi_ub2_i(:,:,mm))
  end do
  print *

  ! terminate program
  deallocate(psi_ub2_i)
  deallocate(psi_ub1_i)
  deallocate(psi_lb_i)
  deallocate(psi_i)
  deallocate(psi_l)
  deallocate(chi_w_l)
  deallocate(w_i)
  deallocate(w_l)
  deallocate(chi_i)
  deallocate(chi_t)
  deallocate(chi_l)
  deallocate(phi_i)
  deallocate(phi_l)
  deallocate(u_i)
  deallocate(u_l)
  print *, "End of example ###"
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
