all: format simple.run snek.run

#SAN = -g -fsanitize=address
#SAN = -g -fsanitize=memory

%.bin: %*.cpp
	clang++ -std=c++1z -Wall -Wextra -Wparentheses -Wno-dangling-else -lncurses -ltinfo -g  -lncurses -ltinfo -o $@ $^ Console-IO/ioconsole.cpp $(SAN)

.PHONY: %.run format
%.run: %.bin
	./$<

.PHONY: format
format:
	@find|egrep '.*[.](cpp|hpp|cxx|hxx|cc|hc|c++|h++|c|h|tpp|txx)$$'|sed 's/[] ()'\''\\[&;]/\\&/g'|xargs clang-format -i ../ioconsole.cpp ../ioconsole.hpp
	@echo "reformatted code"
