```assembly
Main: 	addi $t0, $0, 32 //int size = 32;
		add $s0, $0, $0
		add $s1, $0, $0
		add $s2, $0, $0 // int hotDay, coldDay……
		//int tempArray[]
		
		add $a0, $0, $t1 //t1 stores the address of the array
		add $a1, $0, $t0
		addi $a2, $0, 1
		jal countArray
		add $s0, $s0, $v0
		addi $a2, $0, -1
		jal countArray
		add $s1, $s1, $v0
		addi $a2, $0, 0
		jal countArray
		add $s2, $s2, $v0
		//main finished
countArray:
		addi $sp, $sp, -4
		sw $ra, 0($sp) //initialization
		
		addi $s3, $0, 0 //cnt
		sll $s4, $a1, 2 //i * 4
		addi $s5, $0, -1 //temp = -1
		addi $s6, $0, 1 // temp2 = 1
	Loop:
		addi $s4, $s4, -4 //numElements -1
		beq $s4, $s5 Exit //while i >= 0
		beq $a2, $s5 case1
		beq $a3, $s6 case2
		j case3
	case1: 
		addi $t0, $a0, $s4
		lw $a3, 0($t0) //pass x to the function
		jal hot
		beq $s6, $v0 Add
		j Loop
	case2:
		addi $t0, $a0, $s4
		lw $a3, 0($t0) //pass x to the function
		jal cold
		beq $s6, $v0 Add
		j Loop
	case3:
		addi $t0, $a0, $s4
		lw $a3, 0($t0) //pass x to the function
		jal comfort
		beq $s6, $v0 Add
		j Loop
	Add:
		addi $s3, $s3, 1
		j Loop
	Exit:
		lw $ra, 0($sp)
		addi $sp, $sp, 4 //return
		jr $ra
hot:
		slti $t0, $a3, 30
		beq $t0, $0 return1
		addi $v0, $0, 0
		jr $ra
	return1:
		addi $v0, $0, 0
		jr $ra
cold:
		slti $v0, $a3, 6
		jr $ra
comfort:
		slti $t0, $a3, 30
		addi $t1, $0, 1
		beq $t0, $t1 Next
		addi $v0, $0, 0
		jr $ra
	Next:
		slti $t0, $a3, 6
		addi $t1, $0, 1
		beq $t0, $t1 return0
		addi $v0, $0, 1
		jr $ra
	return0:
		addi $v0, $0, 0
		jr $ra
		
		
		
```

