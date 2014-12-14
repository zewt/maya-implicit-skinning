/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
#include "memory_debug.hpp"

void Memory_stack::push(const void* address, size_t size, const char* name, mem_kind type){
	if(n < stack_size){
		entries[n] = mem_s(address, size, name, type);
		n++;
	} else {
		realloc();
		push( address, size, name, type);
	}
}
	
void Memory_stack::pop(const void* address){
	for(int i = 0; i < n; i++){
		if(entries[i].address == address){
			entries[i] = entries[--n];
			break;
			}
	}
}

void Memory_stack::print(){
	printf("address\tkind\tsize(bytes)\tname\n");
	size_t total = 0;
	for(int i = 0; i < n; i++){
		mem_s m = entries[i];
		printf("%p\t%s\t%d\t%s\n", m.address, (m.kind == CUDA_ARRAY)?"CA":"LM", static_cast<int>(m.size), m.name);
		total += m.size;
	}
	printf("%d elements\t\toccupancy: %d bytes\n",n,static_cast<int>(total));
}

void Memory_stack::realloc(){
	mem_s* new_entries = new mem_s[2*stack_size];
	for(int i = 0; i < stack_size; i++){
		new_entries[i] = entries[i];
	}
	delete[] entries;
	entries = new_entries;
}

int Memory_stack::n = 0;
int Memory_stack::stack_size = DEFAULT_STACK_SIZE;
Memory_stack::mem_s* Memory_stack::entries = new Memory_stack::mem_s[DEFAULT_STACK_SIZE];

