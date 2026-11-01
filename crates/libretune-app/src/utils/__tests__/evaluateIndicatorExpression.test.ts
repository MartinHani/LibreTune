// Added 2026-11-01 in the HaniCore K-line fork of LibreTune (see FORK.md).
import { describe, it, expect } from 'vitest';
import { evaluateIndicatorExpression, evalExpression } from '../evaluateIndicatorExpression';

describe('evaluateIndicatorExpression (no eval)', () => {
  it('evaluates the comparisons indicator tiles use', () => {
    const live = { etbCalResult: 3, etbFailPhase: 0, nvmNotBurned: 1 };
    expect(evaluateIndicatorExpression(' etbCalResult == 3 ', live)).toBe(true);
    expect(evaluateIndicatorExpression('etbCalResult == 3 && etbFailPhase > 0', live)).toBe(false);
    expect(evaluateIndicatorExpression('(etbCalResult == 2 || etbCalResult == 3) && nvmNotBurned', live)).toBe(true);
    expect(evaluateIndicatorExpression('{ etbCalResult != 3 }', live)).toBe(false);
  });

  it('keeps arithmetic, shifts, bit masks and the ternary', () => {
    expect(evalExpression('(lambdaStatus >> 2) & 7', { lambdaStatus: 0b10100 })).toBe(5);
    expect(evalExpression('a + b * 2 - 1', { a: 1, b: 3 })).toBe(6);
    expect(evalExpression('x | 0x10', { x: 1 })).toBe(17);
    expect(evalExpression('rpm > 1000 ? 1 : 2', { rpm: 500 })).toBe(2);
    expect(evalExpression('!flag', {})).toBe(1);
  });

  it('reads an unknown name as 0 and refuses garbage', () => {
    expect(evaluateIndicatorExpression('missing > -1', {})).toBe(true);
    expect(evaluateIndicatorExpression('missing > 0', {})).toBe(false);
    expect(evaluateIndicatorExpression('a == ; b', { a: 1 })).toBe(false);
    expect(evaluateIndicatorExpression('a ==', { a: 1 })).toBe(false);
  });

  it('prefers the live value over a constant of the same name', () => {
    expect(evaluateIndicatorExpression('etbCalResult == 3', { etbCalResult: 3 }, { etbCalResult: 0 })).toBe(true);
  });
});
