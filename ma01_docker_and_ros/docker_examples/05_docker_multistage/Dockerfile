# Stage 1
FROM gcc:7.2 AS compiler
COPY src /src
RUN gcc -static -o /src/hello_multistaged /src/hello_multistaged.c && strip -R .comment -s /src/hello_multistaged

# Stage 2
FROM scratch
COPY --from=compiler /src/hello_multistaged .
CMD ["./hello_multistaged"]